#ifndef __ns_sat_geometry_h__
#define __ns_sat_geometry_h__

#include <math.h>
#include "basic.h"

// Library of routines involving satellite geometry
class SatGeometry {
public:
	SatGeometry(double latborder);
	SatGeometry();
	static double distance(coordinate, coordinate);              
	static void spherical_to_cartesian(double, double, double,
	    double &, double &, double &);
	static double propdelay(coordinate, coordinate);
	static double get_latitude(coordinate);
	static double get_longitude(coordinate, int NOW);
	static double get_radius(coordinate a) { return a.r;}
	static double get_altitude(coordinate);
	static double check_elevation(coordinate, coordinate, double);
	static int are_satellites_mutually_visible(coordinate, coordinate);
	static bool inPolar(coordinate a);
private:
	static double _latborder;
};




class SatPosition {
 public:
	SatPosition();
	int type() { return type_; }
	double period() { return period_; }
	virtual coordinate coord(int NOW) = 0; 

	// configuration parameters
 protected:
	coordinate initial_;
	double period_;
	int type_;
};

class PolarSatPosition : public SatPosition {
 public:
	PolarSatPosition(double = 1000, double = 90, double = 0, double = 0, 
            double = 0);
	PolarSatPosition(double altitude, double Inc, double Lon, 
    double Alpha, int Plane, int index);
	virtual coordinate coord(int NOW);
	void set(double Altitude, double Lon, double Alpha, double inclination=90); 
	bool isascending(int NOW);
	PolarSatPosition* next() { return next_; }
	int plane() { return plane_; }

 protected:
    PolarSatPosition* next_;    // Next intraplane satellite
	int plane_;  // Orbital plane that this satellite resides in
	int index_;
	double inclination_; // radians
};


class TermSatPosition : public SatPosition {
 public:
	TermSatPosition(double = 0, double = 0);
	//virtual coordinate coord();
	void set(double latitude, double longitude);
 protected:
};

#endif // __ns_sat_geometry_h__
