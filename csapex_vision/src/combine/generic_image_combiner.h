#ifndef GENERIC_IMAGE_COMBINER_H
#define GENERIC_IMAGE_COMBINER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/variadic_io.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#if ((BOOST_VERSION >> 20) & 0xF) >= 1 && ((BOOST_VERSION >> 8) & 0xFFF) >= 49
#include <boost/spirit/include/support_utree.hpp>
#endif

#include <boost/smart_ptr/make_shared.hpp>
#include <opencv2/opencv.hpp>

namespace csapex
{
struct VariableMap
{
public:
    cv::Mat& get(std::string const& name)
    {
        return _symbols[name];
    }

private:
    std::map<std::string, cv::Mat> _symbols;
};

struct AbstractExpression;
typedef std::shared_ptr<AbstractExpression> Ptr;

struct AbstractExpression
{
    virtual ~AbstractExpression()
    {
    }
    virtual cv::Mat evaluate(VariableMap& vm) const = 0;
    virtual std::ostream& print(std::ostream& os) const = 0;

    friend std::ostream& operator<<(std::ostream& os, AbstractExpression const& e)
    {
        return e.print(os);
    }

protected:
    AbstractExpression()
    {
    }
};

struct Expression : AbstractExpression
{
    Expression()
    {
    }

    template <typename E>
    Expression(E const& e) : e_(make_from(e))
    {
    }  // cloning the expression

    bool valid() const
    {
        return (bool)e_;
    }
    cv::Mat evaluate(VariableMap& vm) const
    {
        apex_assert(e_);
        return e_->evaluate(vm);
    }

    // special purpose overload to avoid unnecessary wrapping
    friend Ptr make_from(Expression const& t)
    {
        return t.e_;
    }

private:
    Ptr e_;
    virtual std::ostream& print(std::ostream& os) const
    {
        return os << "Expression(" << *e_ << ")";
    }
};

template <typename Expr>  // general purpose, static Expression cloner
static Ptr make_from(Expr const& t)
{
    return std::make_shared<Expr>(t);
}

struct FunctionExpression : AbstractExpression
{
    FunctionExpression()
    {
    }

    template <typename S, typename A>
    FunctionExpression(S op, A const& arg) : op_(op)
    {
        args_.push_back(arg);
    }

    template <typename A>
    FunctionExpression(const std::vector<char>& op, A const& arg) : args_(arg)
    {
        op_.insert(op_.begin(), op.begin(), op.end());
    }

    cv::Mat evaluate(VariableMap& vm) const
    {
        if (op_ == "abs") {
            return cv::abs(args_[0].evaluate(vm));

        } else if (op_ == "log") {
            cv::Mat r;
            cv::log(args_[0].evaluate(vm), r);
            return r;

        } else if (op_ == "exp") {
            cv::Mat r;
            cv::exp(args_[0].evaluate(vm), r);
            return r;

        } else if (op_ == "pow") {
            if (args_.size() != 2) {
                throw std::runtime_error(std::string("'pow'' needs 2 parameters"));
            }

            cv::Mat first = args_[0].evaluate(vm);
            cv::Mat second = args_[1].evaluate(vm);
            if (second.rows != 1 || second.cols != 1) {
                throw std::runtime_error(std::string("'pow'' needs its second parameter to be a number"));
            }

            double power = second.at<double>(0, 0);

            cv::Mat r;
            cv::pow(first, power, r);
            return r;

        } else if (op_ == "sqrt") {
            if (args_.size() != 1) {
                throw std::runtime_error(std::string("'sqrt'' needs 1 parameter"));
            }

            cv::Mat r;
            cv::sqrt(args_[0].evaluate(vm), r);
            return r;

        } else if (op_ == "min") {
            if (args_.size() == 1) {
                return args_[0].evaluate(vm);
            }
            cv::Mat res = cv::min(args_[0].evaluate(vm), args_[1].evaluate(vm));

            for (std::size_t i = 2; i < args_.size(); ++i) {
                cv::min(res, args_[i].evaluate(vm), res);
            }

            return res;

        } else if (op_ == "max") {
            if (args_.size() == 1) {
                return args_[0].evaluate(vm);
            }
            cv::Mat res = cv::max(args_[0].evaluate(vm), args_[1].evaluate(vm));

            for (std::size_t i = 2; i < args_.size(); ++i) {
                cv::max(res, args_[i].evaluate(vm), res);
            }

            return res;
        }

        throw std::runtime_error(std::string("unknown function: '") + op_ + "'");
    }

private:
    std::string op_;
    std::vector<Expression> args_;

    std::ostream& print(std::ostream& os) const
    {
        return os << "FunctionExpression("
                  << " " << op_ << "(...) )";
    }
};

struct BinaryExpression : AbstractExpression
{
    BinaryExpression()
    {
    }

    template <typename L, typename R>
    BinaryExpression(char op, L const& l, R const& r) : op_(op), lhs_(make_from(l)), rhs_(make_from(r))
    {
    }

    cv::Mat evaluate(VariableMap& vm) const
    {
        const cv::Mat& a = lhs_->evaluate(vm);
        const cv::Mat& b = rhs_->evaluate(vm);

        bool a_is_num = (a.rows == 1 && a.cols == 1);
        bool b_is_num = (b.rows == 1 && b.cols == 1);

        if (a_is_num && b_is_num) {
            switch (op_) {
                case '+':
                    return a + b;
                case '-':
                    return a - b;
                case '*':
                    return a * b;
                case '/':
                    return a / b;
            }
            throw std::runtime_error(std::string("unknown operation for numbers: '") + op_ + "'");
        }

        if (a_is_num || b_is_num) {
            if (a_is_num) {
                double v = a.at<double>(0, 0);
                cv::Scalar s = cv::Scalar::all(v);
                switch (op_) {
                    case '+':
                        return s + b;
                    case '-':
                        return s - b;
                    case '*':
                        return v * b;
                }

            } else {
                double v = b.at<double>(0, 0);
                cv::Scalar s = cv::Scalar::all(v);
                switch (op_) {
                    case '+':
                        return a + s;
                    case '-':
                        return a - s;
                    case '*':
                        return a * v;
                    case '/':
                        return 1.0 / v * a;
                }
            }
            throw std::runtime_error(std::string("unknown operation for mixed operators: '") + op_ + "'");
        }

        switch (op_) {
            case '+':
                return a + b;
            case '-':
                return a - b;
            case '&':
                return a & b;
            case '|':
                return a | b;
            case '^':
                return a ^ b;
        }

        throw std::runtime_error(std::string("unknown operation for images: '") + op_ + "'");
    }

private:
    char op_;
    Ptr lhs_, rhs_;

    std::ostream& print(std::ostream& os) const
    {
        return os << "BinaryExpression(" << *lhs_ << " " << op_ << " " << *rhs_ << ")";
    }
};

struct UnaryExpression : AbstractExpression
{
    UnaryExpression()
    {
    }

    template <typename R>
    UnaryExpression(char op, R const& r) : op_(op), rhs_(make_from(r))
    {
    }

    cv::Mat evaluate(VariableMap& vm) const
    {
        const cv::Mat& b = rhs_->evaluate(vm);

        bool b_is_num = (b.rows == 1 && b.cols == 1);

        if (b_is_num) {
            switch (op_) {
                case '~':
                    return ~b;
            }
            throw std::runtime_error(std::string("unknown operation for numbers: '") + op_ + "'");
        }

        switch (op_) {
            case '~':
                return ~b;
        }

        throw std::runtime_error(std::string("unknown operation for images: '") + op_ + "'");
    }

private:
    char op_;
    Ptr rhs_;

    std::ostream& print(std::ostream& os) const
    {
        return os << "UnaryExpression("
                  << " " << op_ << " " << *rhs_ << ")";
    }
};

struct ConstantExpression : AbstractExpression
{
    cv::Mat value;
    ConstantExpression(double v = 0)
    {
        value = cv::Mat(1, 1, CV_64F);
        value.at<double>(0, 0) = v;
    }

    cv::Mat evaluate(VariableMap& vm) const
    {
        return value;
    }

    virtual std::ostream& print(std::ostream& os) const
    {
        return os << "ConstantExpression(" << value << ")";
    }
};

struct VariableExpression : AbstractExpression
{
    std::string name_;

    cv::Mat evaluate(VariableMap& vm) const
    {
        cv::Mat mat = vm.get(name_);
        if (mat.empty()) {
            throw std::runtime_error(std::string("variable $") + name_ + " is not set");
        }
        return mat;
    }

    virtual std::ostream& print(std::ostream& os) const
    {
        return os << "VariableExpression('" << name_ << "')";
    }
};

class GenericImageCombiner : public csapex::Node, public csapex::VariadicInputs
{
public:
    GenericImageCombiner();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

    csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override;

private:
    void updateFormula();

private:
    csapex::Output* out_;

    Expression e;
};

}  // namespace csapex

#endif  // GENERIC_IMAGE_COMBINER_H
