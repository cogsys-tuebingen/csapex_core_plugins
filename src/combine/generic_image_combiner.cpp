/// HEADER
#include "generic_image_combiner.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/fusion/adapted.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::GenericImageCombiner, csapex::Node)

using namespace csapex;
using namespace connection_types;
using namespace vision_plugins;

BOOST_FUSION_ADAPT_STRUCT(VariableExpression, (std::string, name_))


namespace qi    = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace phx   = boost::phoenix;

template <typename Iterator>
struct ExpressionParser : qi::grammar<Iterator, Expression(), ascii::space_type>
{
    struct MakeBinaryExpression {
        template<typename,typename,typename> struct result { typedef BinaryExpression type; };

        template<typename C, typename L, typename R>
        BinaryExpression operator()(C op, L const& lhs, R const& rhs) const
        { return BinaryExpression(op, lhs, rhs); }
    };
    struct MakeUnaryExpression {
        template<typename,typename> struct result { typedef UnaryExpression type; };

        template<typename C, typename R>
        UnaryExpression operator()(C op, R const& rhs) const
        { return UnaryExpression(op, rhs); }
    };

    phx::function<MakeBinaryExpression> makebinary;
    phx::function<MakeUnaryExpression> makeunary;

    ExpressionParser() : ExpressionParser::base_type(expression)
    {
        using namespace qi;
        using qi::_1;
        using qi::_2;

        expression =
                bi_expr                                   [ _val = _1]
                ;

        bi_expr =
                primary_expr                              [ _val = _1 ]
                >> *(char_("-+|&*/^") >> primary_expr)    [ _val = makebinary(_1, _val, _2)]
                ;

        un_expr =
                *(char_("~") >> primary_expr)             [ _val = makeunary(_1, _2)]
                ;

        primary_expr =
                  ( '(' > expression > ')' )              [ _val = _1 ]
                | ( '|' > expression > '|' )              [ _val = _1 ]
                | constant                                [ _val = _1 ]
                | variable                                [ _val = _1 ]
                ;

        constant = double_ | int_;
        variable = '$' >> lexeme [ *~char_(" -+|&*/^~()") ];

        BOOST_SPIRIT_DEBUG_NODE(expression);
        BOOST_SPIRIT_DEBUG_NODE(bi_expr);

        BOOST_SPIRIT_DEBUG_NODE(primary_expr);
        BOOST_SPIRIT_DEBUG_NODE(constant);
        BOOST_SPIRIT_DEBUG_NODE(variable);
    }

    qi::rule<Iterator, Expression(), ascii::space_type> expression;
    qi::rule<Iterator, Expression(), ascii::space_type> bi_expr;
    qi::rule<Iterator, Expression(), ascii::space_type> un_expr;

    qi::rule<Iterator, Expression(), ascii::space_type> primary_expr;
    qi::rule<Iterator, ConstantExpression(), ascii::space_type> constant;
    qi::rule<Iterator, VariableExpression(), ascii::space_type> variable;
    qi::rule<Iterator, std::string() , ascii::space_type> string;
};

GenericImageCombiner::GenericImageCombiner()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));
    addParameter(param::ParameterFactory::declareText("script", "$1 ^ $2"),
                 boost::bind(&GenericImageCombiner::updateFormula, this));
}

void GenericImageCombiner::updateFormula()
{
    std::string script = param<std::string>("script");

    typedef std::string::const_iterator iterator_type;

    std::string::const_iterator iter = script.begin();
    std::string::const_iterator end = script.end();

    ExpressionParser<std::string::const_iterator> p;

    bool r = qi::phrase_parse(iter,end,p,ascii::space,e);

    if (r && iter == end) {
        setError(false);

    } else {
        std::string rest(iter, end);
        throw std::runtime_error(std::string("Parsing failed at: ") + rest);
    }
}

void GenericImageCombiner::process()
{
    if(!e.valid()) {
        return;
    }
    CvMatMessage::Ptr img1 = i1_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr img2 = i2_->getMessage<CvMatMessage>();
    VariableExpression::get("1") = img1->value;
    VariableExpression::get("2") = img2->value;

    CvMatMessage::Ptr out(new CvMatMessage(img1->getEncoding()));

    out->value = e.evaluate();

    out_->publish(out);
}

void GenericImageCombiner::setup()
{
    setSynchronizedInputs(true);

    i1_ = addInput<CvMatMessage>("image 1");
    i2_ = addInput<CvMatMessage>("image 2");
    out_ = addOutput<CvMatMessage>("combined");
}
