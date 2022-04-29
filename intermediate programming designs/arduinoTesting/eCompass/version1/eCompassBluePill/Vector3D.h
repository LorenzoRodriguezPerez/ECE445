class Vector3D
    {
    public:
        Vector3D() : mArr{} {}
    
        Vector3D(double x, double y, double z)
        {
            mArr[0] = x;
            mArr[1] = y;
            mArr[2] = z;
        }
    
        double dot(const Vector3D& rhs) const
        {
            double out = 0;
            for (int i = 0; i < NUM_DIMENSIONS; ++i)
            {
                out = out + mArr[i] * rhs.mArr[i];
            }
            return out;
        }
    
        Vector3D operator* (double x) const
        {
            Vector3D out;
            for (int i = 0; i < NUM_DIMENSIONS; ++i)
            {
                out.mArr[i] = mArr[i] * x;
            }
            return out;
        }
    
        Vector3D operator- (const Vector3D& x) const
        {
            Vector3D out;
            for (int i = 0; i < NUM_DIMENSIONS; ++i)
            {
                out.mArr[i] = mArr[i] - x.mArr[i];
            }
            return out;
        }
        
        double getX() const {return mArr[0];}
        double getY() const {return mArr[1];}
        double getZ() const {return mArr[2];}
    
    private:
        static const int NUM_DIMENSIONS = 3;
        double mArr[NUM_DIMENSIONS];
    };