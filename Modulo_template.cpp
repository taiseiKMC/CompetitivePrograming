using LL=long long;

template<LL MODULO=1000000007>
class Mod
{
    private:
        LL value;

        const void Normalize()
        {
            value=value<0?(value%MODULO+MODULO):(value%MODULO);
        }

    public:
        Mod<MODULO>():value(0){}
        Mod<MODULO>(const LL &val)
        {
            value=val;
            Normalize();
        }
        
        explicit operator LL () const
        {
            return value;
        }

        const Mod<MODULO> operator -() const
        {
            return Mod<MODULO>(MODULO - value);
        }
        const Mod<MODULO> operator +(const Mod<MODULO> &rhs) const
        {
            return Mod<MODULO>(value + rhs.value);
        }
        const Mod<MODULO> operator -(const Mod<MODULO> &rhs) const
        {
            return Mod<MODULO>(value + (-rhs).value);
        }
        const Mod<MODULO> operator *(const Mod<MODULO> &rhs) const
        {
            return Mod<MODULO>(value * rhs.value);
        }
        Mod<MODULO> &operator +=(const Mod<MODULO> &rhs)
        {
            return *this = *this + rhs;
        }
        Mod<MODULO> &operator -=(const Mod<MODULO> &rhs)
        {
            return *this = *this - rhs;
        }
        Mod<MODULO> &operator *=(const Mod<MODULO> &rhs)
        {
            return *this = *this * rhs;
        }


        Mod<MODULO> pow(LL p) const;

        Mod<MODULO> inv() const
        {
            return pow(MODULO-2);
        }

        const Mod<MODULO> operator /(const Mod<MODULO> &rhs) const
        {
            return *this * rhs.inv();
        }
        Mod<MODULO> &operator /=(const Mod<MODULO> &rhs)
        {
            return *this = *this / rhs;
        }

        const bool operator <(const Mod<MODULO> &rhs) const
        {
            return value < rhs.value;
        }
        const bool operator ==(const Mod<MODULO> &rhs) const
        {
            return value == rhs.value;
        }
};

template<LL MODULO>
Mod<MODULO> Mod<MODULO>::pow(LL p) const
{
    Mod<MODULO> tmp=1, mult=*this;
    while(p)
    {
        if((p&1)>0) tmp*=mult;
        p>>=1;
        mult*=mult;
    }
    return tmp;
}

namespace std
{
    template<LL MODULO>
    ostream& operator<<(ostream& os, const Mod<MODULO> mod)
    {
        os<<(LL)mod;
        return os;
    }
};

