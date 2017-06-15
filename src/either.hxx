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
    static either<L,R> left (L l) { return either(&l,NULL); }
    static either<L,R> right(R r) { return either(NULL,&r); }
    bool isLeft () const { return (_l != NULL); }
    bool isRight() const { return (_r != NULL); }
    operator bool() { return isRight(); }
    operator R() {
        if(isRight())
        {
            return *_r;
        } else {
            throw std::runtime_error(*_l);
        } }
    L fromLeft  () const { assert(isLeft ()); return L(*_l); }
    R fromRight () const { assert(isRight()); return R(*_r); }
    ~either() {
        if (_l != NULL) delete _l;
        if (_r != NULL) delete _r;
    }
private:
    L* _l; R* _r;
    either(const L* l,const R* r) : _l(l != NULL ? new L(*l) : NULL)
                                  , _r(r != NULL ? new R(*r) : NULL) {}
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
