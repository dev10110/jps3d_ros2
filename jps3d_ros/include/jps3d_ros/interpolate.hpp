#pragma once
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

enum InterpolationMode
{
	LINEAR=0,
	PRE,
	POST,
};

double getFrac(double t, double t0, double t1, InterpolationMode M=LINEAR)
{
	    switch (M) {
		    case PRE:
			    return 0.0;
		    case POST:
			    return 1.0;
		    case LINEAR: 
		    default:
			    return (t - t0) / (t1 - t0);
	    }
}


template <typename T, class A>
bool interpolate(T &res, const double t, const std::vector<double> &ts,
                 const std::vector<T, A> &xs, InterpolationMode M=LINEAR) {

  for (std::size_t ind = 0; ind < ts.size() - 1; ++ind) {
    if (ts[ind] <= t && t < ts[ind + 1]) {
	    double frac = getFrac(t, ts[ind], ts[ind+1], M);
	    res = (1 - frac) * xs[ind] + frac * xs[ind + 1];
	    return true;
    }
  }

  return false;
}

template <typename T, class A>
bool interpolate(std::vector<T, A> &res, const std::vector<double> t, const std::vector<double> &ts,
                 const std::vector<T, A> &xs, InterpolationMode M=LINEAR) {

  // assume both t and ts are sorted
  std::size_t t_ind = 0;
  std::size_t ts_ind = 0;

  std::size_t N_t = t.size();
  std::size_t N_ts = ts.size();


  if (N_ts <= 1){
    std::cout << "interp failed: N_ts <= 1" << std::endl;
	  return false;
  }
  if (N_t < 1) {
    std::cout << "interp failed: N_t <= 1" << std::endl;
	  return false;
  }
  
  res.resize(N_t); // clear is not needed, since each element will be replaced

  while (true)
  {

	  if (ts[ts_ind] <= t[t_ind]  && t[t_ind] <= ts[ts_ind+1])
	  {
		  double frac = getFrac(t[t_ind], ts[ts_ind], ts[ts_ind+1], M);
		  res[t_ind] = (1 - frac) * xs[ts_ind] + frac * xs[ts_ind + 1];
		  t_ind += 1;
		  if (t_ind >= N_t) {
			  // filled all the values
			  return true;
		  }
	  }
	  else if ( t[t_ind] >= ts[ts_ind + 1] )
	  {
		  ts_ind +=1 ;
		  if (ts_ind >= N_ts)
		  {
			  // trying to extrapolate
        std::cout << "interp failed: extrapolating" << std::endl;
			  return false;
		  }
	  }
	  else if (  t[t_ind] < ts[ts_ind] )
	  {
		  // trying to pre-extrapolate
      std::cout << "interp failed: pre-extrapolating" << std::endl;
		  return false;
	  }
  }
}

double fmod_floor(double a, double n) {
  return std::fmod(std::fmod(a, n) + n, n);
}

double wrap(double x) { return fmod_floor(x + M_PI, 2 * M_PI) - M_PI; }

double interpolate_angles(double b, double a, double f) {
  // returns a + f * (b - a) but wrapped correctly
  // asssumes radians
  //
  return wrap(a + wrap(b - a) * f);
}
