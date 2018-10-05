/// HEADER
#include "generic_image_combiner.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

/// SYSTEM
#include <boost/fusion/adapted.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

CSAPEX_REGISTER_CLASS(csapex::GenericImageCombiner, csapex::Node)

using namespace csapex;
using namespace connection_types;
using namespace csapex;

BOOST_FUSION_ADAPT_STRUCT(VariableExpression, (std::string, name_))

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace phx = boost::phoenix;

template <typename Iterator>
struct ExpressionParser : qi::grammar<Iterator, Expression(), ascii::space_type>
{
    struct MakeFunctionExpression
    {
        template <typename, typename>
        struct result
        {
            typedef FunctionExpression type;
        };

        template <typename C, typename A>
        FunctionExpression operator()(C op, A const& arg) const
        {
            return FunctionExpression(op, arg);
        }
    };
    struct MakeBinaryExpression
    {
        template <typename, typename, typename>
        struct result
        {
            typedef BinaryExpression type;
        };

        template <typename C, typename L, typename R>
        BinaryExpression operator()(C op, L const& lhs, R const& rhs) const
        {
            return BinaryExpression(op, lhs, rhs);
        }
    };
    struct MakeUnaryExpression
    {
        template <typename, typename>
        struct result
        {
            typedef UnaryExpression type;
        };

        template <typename C, typename R>
        UnaryExpression operator()(C op, R const& rhs) const
        {
            return UnaryExpression(op, rhs);
        }
    };

    phx::function<MakeBinaryExpression> makebinary;
    phx::function<MakeUnaryExpression> makeunary;
    phx::function<MakeFunctionExpression> makefun;

    ExpressionParser() : ExpressionParser::base_type(expression)
    {
        using namespace qi;
        using qi::_1;
        using qi::_2;

        expression = bi_expr[_val = _1];

        expression_list = expression % ',';

        bi_expr = primary_expr[_val = _1] >> *(char_("+|&*/^-") >> primary_expr)[_val = makebinary(_1, _val, _2)];

        un_expr = *(char_("~") >> primary_expr)[_val = makeunary(_1, _2)];

        primary_expr = function_call[_val = _1] | ('(' > expression > ')')[_val = _1] | ('|' > expression > '|')[_val = makefun(std::string("abs"), _1)] | constant[_val = _1] | variable[_val = _1];

        function_call = (+char_("a-zA-Z0-9") >> '(' >> expression_list >> ')')[_val = makefun(_1, _2)];
        constant = double_ | int_;
        variable = '$' >> lexeme[+char_("a-zA-Z0-9")];

        BOOST_SPIRIT_DEBUG_NODE(expression);
        BOOST_SPIRIT_DEBUG_NODE(bi_expr);

        BOOST_SPIRIT_DEBUG_NODE(primary_expr);
        BOOST_SPIRIT_DEBUG_NODE(constant);
        BOOST_SPIRIT_DEBUG_NODE(variable);
    }

    qi::rule<Iterator, Expression(), ascii::space_type> expression;
    qi::rule<Iterator, Expression(), ascii::space_type> bi_expr;
    qi::rule<Iterator, Expression(), ascii::space_type> un_expr;
    qi::rule<Iterator, std::vector<Expression>(), ascii::space_type> expression_list;

    qi::rule<Iterator, Expression(), ascii::space_type> primary_expr;
    qi::rule<Iterator, ConstantExpression(), ascii::space_type> constant;
    qi::rule<Iterator, VariableExpression(), ascii::space_type> variable;
    qi::rule<Iterator, FunctionExpression(), ascii::space_type> function_call;
    qi::rule<Iterator, std::string(), ascii::space_type> string;
};

GenericImageCombiner::GenericImageCombiner()
{
}

void GenericImageCombiner::updateFormula()
{
    std::string script = readParameter<std::string>("script");

    std::string::const_iterator iter = script.begin();
    std::string::const_iterator end = script.end();

    ExpressionParser<std::string::const_iterator> p;

    bool r = qi::phrase_parse(iter, end, p, ascii::space, e);

    if (r && iter == end) {
        node_modifier_->setNoError();

    } else {
        std::string rest(iter, end);
        throw std::runtime_error(std::string("Parsing failed at: ") + rest);
    }
}

void GenericImageCombiner::process()
{
    if (!e.valid()) {
        return;
    }

    std::vector<InputPtr> inputs = node_modifier_->getMessageInputs();
    if (inputs.empty()) {
        return;
    }

    VariableMap vm;

    CvMatMessage::ConstPtr img_0;

    for (std::size_t i = 0; i < inputs.size(); i++) {
        Input* in = inputs[i].get();

        if (!msg::hasMessage(in)) {
            vm.get(std::to_string(i + 1)) = cv::Mat();
            continue;
        }

        CvMatMessage::ConstPtr img_i = msg::getMessage<CvMatMessage>(in);

        if (!img_0) {
            img_0 = img_i;
        }

        cv::Mat f_i;
        img_i->value.copyTo(f_i);

        if (f_i.channels() == 1) {
            f_i.convertTo(f_i, CV_32FC1);
        } else if (f_i.channels() == 3) {
            f_i.convertTo(f_i, CV_32FC3);
        } else {
            std::stringstream msg;
            msg << "Image " << i << ": images with " << f_i.channels() << " channels not yet supported";
            throw std::runtime_error(msg.str());
        }

        vm.get(std::to_string(i + 1)) = f_i;
    }

    if (img_0) {
        CvMatMessage::Ptr out(new CvMatMessage(img_0->getEncoding(), img_0->frame_id, img_0->stamp_micro_seconds));

        cv::Mat r = e.evaluate(vm);
        if (img_0->value.channels() == 1) {
            r.convertTo(out->value, CV_8UC1);
        } else if (img_0->value.channels() == 3) {
            r.convertTo(out->value, CV_8UC3);
        }
        msg::publish(out_, out);
    }
}

void GenericImageCombiner::setup(NodeModifier& node_modifier)
{
    setupVariadic(node_modifier);

    out_ = node_modifier.addOutput<CvMatMessage>("combined");
}

void GenericImageCombiner::setupParameters(Parameterizable& parameters)
{
    setupVariadicParameters(parameters);

    parameters.addParameter(csapex::param::factory::declareText("script",
                                                                csapex::param::ParameterDescription("An opencv-style script to combine the images.\n"
                                                                                                    "Inputs are mapped to variables $1 ... $n\n"
                                                                                                    "Valid operators are +, -, *, /, ^, &, |\n"
                                                                                                    "Functions: abs, min, max, exp, log, pow, sqrt"),
                                                                "$1 ^ $2"),
                            std::bind(&GenericImageCombiner::updateFormula, this));
}

Input* GenericImageCombiner::createVariadicInput(TokenDataConstPtr type, const std::string& label, bool /*optional*/)
{
    return VariadicInputs::createVariadicInput(makeEmpty<CvMatMessage>(), label.empty() ? "Image" : label, getVariadicInputCount() == 0 ? false : true);
}
