#ifndef GENERIC_IMAGE_COMBINER_H
#define GENERIC_IMAGE_COMBINER_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#if ((BOOST_VERSION >> 20) & 0xF) >= 1 && ((BOOST_VERSION >> 8) & 0xFFF) >= 49
#include <boost/spirit/include/support_utree.hpp>
#endif

#include <boost/smart_ptr/make_shared.hpp>
#include <opencv2/opencv.hpp>

namespace csapex {

struct AbstractExpression;
typedef boost::shared_ptr<AbstractExpression> Ptr;

struct AbstractExpression {
    virtual ~AbstractExpression() {}
    virtual cv::Mat evaluate() const = 0;
    virtual std::ostream& print(std::ostream& os) const = 0;

    friend std::ostream& operator<<(std::ostream& os, AbstractExpression const& e)
    { return e.print(os); }

protected: AbstractExpression() {}
};

template <typename Expr> // general purpose, static Expression cloner
static Ptr make_from(Expr const& t) { return boost::make_shared<Expr>(t); }

struct BinaryExpression : AbstractExpression
{
    BinaryExpression() {}

    template<typename L, typename R>
    BinaryExpression(char op, L const& l, R const& r)
        : op_(op), lhs_(make_from(l)), rhs_(make_from(r))
    {}

    cv::Mat evaluate() const {
        const cv::Mat &a = lhs_->evaluate();
        const cv::Mat &b = rhs_->evaluate();

        bool a_is_num = (a.rows== 1 && a.cols == 1);
        bool b_is_num = (b.rows== 1 && b.cols == 1);

        if(a_is_num && b_is_num) {
            switch(op_) {
            case '+': return a+b;
            case '-': return a-b;
            case '*': return a*b;
            case '/': return a/b;
            }
            throw std::runtime_error(std::string("unknown operation for numbers: '") + op_ + "'");
        }


        if(a_is_num || b_is_num) {
            if(a_is_num) {
                double v = a.at<double>(0,0);
                switch(op_) {
                case '+': return v+b;
                case '-': return v-b;
                case '*': return v*b;
                }

            } else {
                double v = b.at<double>(0,0);
                switch(op_) {
                case '+': return a+v;
                case '-': return a-v;
                case '*': return v*a;
                case '/': return 1.0/v * a;
                }
            }
            throw std::runtime_error(std::string("unknown operation for mixed operators: '") + op_ + "'");
        }

        switch(op_) {
        case '+': return a+b;
        case '-': return a-b;
        case '&': return a&b;
        case '|': return a|b;
        case '^': return a^b;
        }

        throw std::runtime_error(std::string("unknown operation for images: '") + op_ + "'");
    }

private:
    char op_;
    Ptr lhs_, rhs_;

    std::ostream& print(std::ostream& os) const
    { return os << "BinaryExpression(" << *lhs_ << " " << op_ << " " << *rhs_ << ")"; }
};

struct UnaryExpression : AbstractExpression
{
    UnaryExpression() {}

    template<typename R>
    UnaryExpression(char op, R const& r)
        : op_(op), rhs_(make_from(r))
    {}

    cv::Mat evaluate() const {
        const cv::Mat &b = rhs_->evaluate();

        bool b_is_num = (b.rows== 1 && b.cols == 1);

        if(b_is_num) {
            switch(op_) {
            case '~': return ~b;
            }
            throw std::runtime_error(std::string("unknown operation for numbers: '") + op_ + "'");
        }

        switch(op_) {
        case '~': return ~b;
        }

        throw std::runtime_error(std::string("unknown operation for images: '") + op_ + "'");
    }

private:
    char op_;
    Ptr rhs_;

    std::ostream& print(std::ostream& os) const
    { return os << "UnaryExpression(" << " " << op_ << " " << *rhs_ << ")"; }
};

struct ConstantExpression : AbstractExpression {
    cv::Mat value;
    ConstantExpression(double v = 0) {
        value = cv::Mat(1,1, CV_64F);
        value.at<double>(0,0) = v;
    }

    cv::Mat evaluate() const { return value; }

    virtual std::ostream& print(std::ostream& os) const
    { return os << "ConstantExpression(" << value << ")"; }
};

struct VariableExpression : AbstractExpression {
    std::string name_;

    static cv::Mat& get(std::string const& name) {
        static std::map<std::string, cv::Mat> _symbols;
        return _symbols[name];
    }

    cv::Mat evaluate() const { return get(name_); }

    virtual std::ostream& print(std::ostream& os) const
    { return os << "VariableExpression('" << name_ << "')"; }
};

struct Expression : AbstractExpression
{
    Expression() { }

    template <typename E>
    Expression(E const& e) : e_(make_from(e)) { } // cloning the expression

    bool valid() const { return e_; }
    cv::Mat evaluate() const { assert(e_); return e_->evaluate(); }

    // special purpose overload to avoid unnecessary wrapping
    friend Ptr make_from(Expression const& t) { return t.e_; }
private:
    Ptr e_;
    virtual std::ostream& print(std::ostream& os) const
    { return os << "Expression(" << *e_ << ")"; }

};

class GenericImageCombiner : public Node
{
public:
    GenericImageCombiner();

    void setup();
    void allConnectorsArrived();

private:
    void updateFormula();

private:
    ConnectorIn* i1_;
    ConnectorIn* i2_;
    ConnectorOut* out_;

    Expression e;
};

}

#endif // GENERIC_IMAGE_COMBINER_H
