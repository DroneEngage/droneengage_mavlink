#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

namespace de {
namespace fcb {
namespace tracking {

class SimpleKalmanFilter {
private:
    double state = 0.0;           // position estimate
    double velocity = 0.0;        // velocity estimate
    double covariance = 1.0;      // estimate uncertainty
    
    // Tuning parameters
    double process_noise_q = 0.01;   // how much we expect motion to change
    double measurement_noise_r = 0.1; // sensor noise level
    
    bool initialized = false;

public:
    SimpleKalmanFilter(double q = 0.01, double r = 0.1) 
        : process_noise_q(q), measurement_noise_r(r) {}
    
    void setParameters(double q, double r) {
        process_noise_q = q;
        measurement_noise_r = r;
    }
    
    double update(double measurement, double dt) {
        if (dt <= 0.0) {
            return measurement;
        }
        
        if (!initialized) {
            state = measurement;
            velocity = 0.0;
            covariance = 1.0;
            initialized = true;
            return state;
        }
        
        // Predict step
        state += velocity * dt;
        covariance += process_noise_q;
        
        // Update step
        double kalman_gain = covariance / (covariance + measurement_noise_r);
        double innovation = measurement - state;
        
        state += kalman_gain * innovation;
        velocity += kalman_gain * (innovation / dt - velocity);
        covariance *= (1.0 - kalman_gain);
        
        return state;
    }
    
    double getVelocity() const { return velocity; }
    double getState() const { return state; }
    double getCovariance() const { return covariance; }
    
    void reset() {
        initialized = false;
        state = 0.0;
        velocity = 0.0;
        covariance = 1.0;
    }
};

} // namespace tracking
} // namespace fcb
} // namespace de

#endif
