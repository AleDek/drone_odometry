#ifndef _FILTERS_H_
#define _FILTERS_H_


# define M_PI           3.14159265358979323846  /* pi */

typedef double num;

class LPF {

    private:
        int N;
        num * y_old;
        num alpha;

    public: 
        LPF(int size, num f_cutoff, num Ts){
            N = size;
            y_old = new num[N];
            num tau = 1.00/(2.00*M_PI*f_cutoff) ;
            alpha = Ts / (tau + Ts);
            for(int i =0;i<N;i++) y_old[i]=0.00;
        }

        ~LPF(){
            delete [] y_old;
        }

        void filter(num* u, num* y){
            for(int i =0;i<N;i++){
                y[i] = alpha * u[i] + (1 - alpha) * y_old[i];
                y_old[i] = y[i]; // Update previous output
            }
        }

};

class DownsamplerMean { //at k, mean or k ... k.N 
    private :
        int _N;
        int _k;
        num _m ;
        num _y;
        bool _period;
        // num * mem;

    public :
        DownsamplerMean(int N = 20){
            _N = N;
            // mem = new num[N];
            _m =0.00;
            _y =0.00;
            _k =0;
            // for(int i =0;i<_N;i++) mem[i]=0.00;

        }

        void setup(int N){
            _N = N;
            _m =0.00;
            _y =0.00;
            _k =0;
        }

        double filter(double u){
            _m+=u;
            _period =false;
            if( ++_k%_N == 0){
                _y =_m/num(_N);
                _m =0.00;
                _period =true;
            }
            return _y;
        }

        bool is_period(){
            return _period;
        }
};


class DownsamplerMeanInterp { //at k, mean or k ... k.N 
    private :
        int _N;
        int _k;
        num _m;
        num _y;
        num _delta;
        num _mo;
        num _y0;
        // num * mem;

    public :
        DownsamplerMeanInterp(int N = 20){
            _N = N;
            // mem = new num[N];
            _m = _mo = 0.00;
            _y = _y0 = 0.00;
            _k =0;
            _delta = 00.00;
            // for(int i =0;i<_N;i++) mem[i]=0.00;
           
        }

        void setup(int N){
             _N = N;
            _m = _mo = 0.00;
            _y = _y0 = 0.00;
            _k =0;
            _delta = 00.00;
        }

        double filter(double u){
            _m+=u;
            int j = ++_k%_N;
            if( j == 0 ){
                _m = _m/num(_N);
                _delta = (_m - _mo)/(num(_N) - 1.00);
                _y0 = _y;
                _mo = _m;
                _m =0.00;
            }
            return _y =_y0+ double(j)*_delta;
        }
};


class MA_scalar {
    private:
        int _N;
        int _k;
        num * _mem;
        num _m;

    
    public: 
        MA_scalar(int N = 5){
            _N = N;
            _m =0.00;
            _mem = new num[_N];
            for(int i =0;i<_N;i++) _mem[i]=0.00;

        }

        ~MA_scalar(){
            delete [] _mem;
        }

        void setup(int N){
            _N = N;
            _m =0.00;
            _mem = new num[_N];
            for(int i =0;i<_N;i++) _mem[i]=0.00;
        }

        double filter(double u){
            _m =0.00;
            for(int i =_N-1;i >0;i--){
                _mem[i] = _mem[i-1];
                _m +=_mem[i];
            }
            _mem[0] = u;
            _m +=_mem[0];
            _m/=num(_N);
            return _m;
        }

};


class MA {

    private :
        int N;
        int M_sample;
        num** ma_state;

    public :

        MA(int size, int m_sample){
            N = size; //r
            M_sample = m_sample; //c
            ma_state = new num*[N];
            for(int i =0;i<N;i++) {
                ma_state[i] = new num[M_sample];
            }

            for(int i =0;i<N;i++) {
                for(int j =0;j<M_sample;j++) ma_state[i][j] =0.00;
            }

        }

        ~MA(){
            for(int i =0;i<N;i++) {
                delete [] ma_state[i];
            }
            delete [] ma_state;
        }

        void filter(num* u, num* y){
            for(int i =0;i<N;i++) {
                y[i] =0.00;
                for(int j =0;j<M_sample-2 ;j++){
                    y[i]+= ma_state[i][j];
                    ma_state[i][j] = ma_state[i][j+1] ;
                }
                y[i]+=u[i];
                ma_state[i][M_sample-1] = u[i];

                y[i] = y[i] /num(M_sample);
            }
        
        }

};





#endif //_FILTERS_H_