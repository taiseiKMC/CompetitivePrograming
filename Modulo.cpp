
class Mod
{
    private:
        static const long long MODULO = 1e9+7;
        long long value;

        const void Normalize()
        {
            value=value<0?(value%MODULO+MODULO):(value%MODULO);
        }

    public:
        Mod():value(0){}
        Mod(const long long &val)
        {
            value=val;
            Normalize();
        }
        
        explicit operator long long () const
        {
            return value;
        }

        const Mod operator -() const
        {
            return Mod(MODULO - value);
        }
        const Mod operator +(const Mod &rhs) const
        {
            return Mod(value + rhs.value);
        }
        const Mod operator -(const Mod &rhs) const
        {
            return Mod(value + (-rhs).value);
        }
        const Mod operator *(const Mod &rhs) const
        {
            return Mod(value * rhs.value);
        }
        Mod &operator +=(const Mod &rhs)
        {
            return *this = *this + rhs;
        }
        Mod &operator -=(const Mod &rhs)
        {
            return *this = *this - rhs;
        }
        Mod &operator *=(const Mod &rhs)
        {
            return *this = *this * rhs;
        }


        Mod pow(long long p) const;

        Mod inv() const
        {
            return pow(MODULO-2);
        }

        const Mod operator /(const Mod &rhs) const
        {
            return *this * rhs.inv();
        }
        Mod &operator /=(const Mod &rhs)
        {
            return *this = *this / rhs;
        }
};

Mod Mod::pow(long long p) const
{
    Mod tmp=1, mult=*this;
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
    ostream& operator<<(ostream& os, const Mod mod)
    {
        os<<(long long)mod;
        return os;
    }
};

class Factorial
{
    private:
        vector<Mod> ary;
    public:
        explicit Factorial(const size_t size):ary(vector<Mod>(size))
        {
            ary[0]=1;
            for(size_t i=1;i<size;i++)
                ary[i]=ary[i-1]*i;
        }

        size_t size() const {   return ary.size();  }

        Mod operator[] (const int id) const
        {
            return ary[id];
        }
};
class FactorialInv
{
    private:
        vector<Mod> ary;
    public:
        explicit FactorialInv(const Factorial &fact):ary(vector<Mod>(fact.size()))
        {
            for(size_t i=0;i<ary.size();i++)
                ary[i]=fact[i].inv();
        }

        //FactorialInv& operator=(FactorialInv&&)=default;

        Mod operator[] (const int id) const
        {
            return ary[id];
        }
};

class Combination
{
    private:
        const Factorial *fact;
        const FactorialInv *fact_inv;
    public:
        Combination(const Factorial &fact_, const FactorialInv &fact_inv_):fact(&fact_),fact_inv(&fact_inv_)
        {}

        Mod operator()(const int n, const int m) const
        {
            return (*fact)[n] * (*fact_inv)[m] * (*fact_inv)[n-m];
        }
};
