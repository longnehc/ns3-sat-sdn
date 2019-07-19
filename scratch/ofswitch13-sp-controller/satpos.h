#ifndef __ns_sat_geometry_h__
#define __ns_sat_geometry_h__

#include <math.h>

// Various constants
#define PI 3.1415926535897
#define MU 398601.2 // Greek Mu (km^3/s^2)
#define LIGHT 299793 // km/s
#define EARTH_PERIOD 86164 // seconds
#define EARTH_RADIUS 6378  // km
#define GEO_ALTITUDE 35786 // km
#define ATMOS_MARGIN 150 // km

#define DEG_TO_RAD(x) ((x) * PI/180)
#define RAD_TO_DEG(x) ((x) * 180/PI)
#define DISTANCE(s_x, s_y, s_z, e_x, e_y, e_z) (sqrt((s_x - e_x) * (s_x - e_x) \
                + (s_y - e_y) * (s_y - e_y) + (s_z - e_z) * (s_z - e_z)))

struct coordinate {
        double r;        // km
        double theta;    // radians
        double phi;      // radians
};

// Library of routines involving satellite geometry
class SatGeometry {
public:
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

};


// Position types
#define POSITION_SAT 1
#define POSITION_SAT_POLAR 2
#define POSITION_SAT_GEO 3
#define POSITION_SAT_TERM 4

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


#endif // __ns_sat_geometry_h__
