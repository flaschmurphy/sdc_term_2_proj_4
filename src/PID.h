#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  // Throttle value
  double throttle;

  // Status marker for Twidle
  double best_err;

  // Gets set to true after the 1st data
  bool is_initialized = false;

  // Tracks the previous cte
  double previous_cte;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Set new values for params
  */
  void SetParams(double Kp, double Ki, double Kd);

};

#endif /* PID_H */
