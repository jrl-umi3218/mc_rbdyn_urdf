#ifndef _either_hxx_
#define _either_hxx_

#include <cassert>
#include <ostream>
#include <functional>

using std::ostream;
using std::function;

template<class L,class R>
class either {
public:
    static either<L,R> left (L l) { return either(&l,NULL,false); }
    static either<L,R> right(R r) { return either(NULL,&r,true); }

    bool isLeft () const { return !_isRight; }
    bool isRight() const { return _isRight; }
    operator bool() { return isRight(); }
    operator R() {
            return fromRight();
        }
    L fromLeft  () const { assert(isLeft ()); return _l; }
    R fromRight () const {
        if(isRight()) {
            return _r;
        } else {
            throw std::runtime_error(_l);
        }
    }
    ~either() {}

private:
    bool _isRight;
    L _l;
    R _r;
    either(const L* l,const R* r, bool isRight)
        : _isRight(isRight)
    {
        if(isRight)
        {
            _r = R(*r);
        }
        else
        {
            _l = L(*l);
        }
    }
};

template<class L,class R>
ostream& operator<<(ostream& out,const either<L,R> &e) {
    if (e.isLeft())  out << "<Left:"  << e.fromLeft()  << ">";
    if (e.isRight()) out << "<Right:" << e.fromRight() << ">";
    return out;
}

template<class L,class R1, class R2>
either<L,R2> Bind(either<L,R1> e,const function<either<L,R2>(R1)>& f) {
    if (e.isLeft()) return either<L,R2>::left(e.fromLeft());
    return f(e.fromRight());
}

template<class L,class R>
either<L,R> Return(R r) {
    return either<L,R>::right(r);
}

template<class L,class R1,class R2>
either<L,R2> operator>>(either<L,R1>e,const function<either<L,R2>(R1)>&f) {
    if (e.isLeft()) return either<L,R2>::left(e.fromLeft());
    return f(e.fromRight());
}

#endif
