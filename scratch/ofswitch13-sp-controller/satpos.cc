#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "satpos.h"
#include <iostream>

using namespace std;

// Returns the distance in km between points a and b
SatGeometry::SatGeometry()
{
    
}
double SatGeometry::distance(coordinate a, coordinate b)
{
    double a_x, a_y, a_z, b_x, b_y, b_z;     // cartesian
	spherical_to_cartesian(a.r, a.theta, a.phi, a_x, a_y, a_z);
	spherical_to_cartesian(b.r, b.theta, b.phi, b_x, b_y, b_z);
        return DISTANCE(a_x, a_y, a_z, b_x, b_y, b_z);
}

void SatGeometry::spherical_to_cartesian(double R, double Theta,
    double Phi, double &X, double &Y, double &Z)
{      
	X = R * sin(Theta) * cos (Phi);
	Y = R * sin(Theta) * sin (Phi);
	Z = R * cos(Theta);
}

// Propagation delay is the distance divided by the speed of light
double SatGeometry::propdelay(coordinate a, coordinate b)
{
	double delay = distance(a, b)/LIGHT;
	return delay;
}

double SatGeometry::get_altitude(coordinate a)
{
        return (a.r - EARTH_RADIUS);
}

// Returns latitude in radians, in the range from -PI/2 to PI/2
double SatGeometry::get_latitude(coordinate a)
{
        return (PI/2 - a.theta);
}

// Returns (earth-centric) longitude corresponding to the position of the node 
// (the input coordinate corresponds to fixed coordinate system, through
// which the Earth rotates, so we have to scale back the effects of rotation).
// The return value ranges from -PI to PI.
double SatGeometry::get_longitude(coordinate coord_, int NOW)
{
        double period = EARTH_PERIOD; // period of earth in seconds
        // adjust longitude so that it is earth-centric (i.e., account
        // for earth rotating beneath).   
        double earth_longitude = fmod((coord_.phi -
           (fmod(NOW, period)/period) * 2*PI), 2*PI);
	    
        // Bring earth_longitude to be within (-PI, PI)
        if (earth_longitude < (-1*PI))
		earth_longitude = 2*PI + earth_longitude;
        if (earth_longitude > PI)
		earth_longitude = (-(2*PI - earth_longitude));
	    if (fabs(earth_longitude) < 0.0001)
		    return 0;   // To avoid trace output of "-0.00"
	    else
		    return (earth_longitude);
}       

// If the satellite is above the elevation mask of the terminal, returns 
// the elevation mask in radians; otherwise, returns 0.
double SatGeometry::check_elevation(coordinate satellite,
    coordinate terminal, double elev_mask_)
{
	double S = satellite.r;  // satellite radius
	double S_2 = satellite.r * satellite.r;  // satellite radius^2
	double E = EARTH_RADIUS;
	double E_2 = E * E;
	double d, theta, alpha;

	d = distance(satellite, terminal);
	if (d < sqrt(S_2 - E_2)) {
		// elevation angle > 0
		theta = acos((E_2+S_2-(d*d))/(2*E*S));
		alpha = acos(sin(theta) * S/d);
		return ( (alpha > elev_mask_) ? alpha : 0);
	} else
		return 0;
}

// This function determines whether two satellites are too far apart
// to establish an ISL between them, due to Earth atmospheric grazing
// (or shadowing by the Earth itself).  Assumes that both satellites nodes
// are at the same altitude.  The line between the two satellites can be
// bisected, and a perpendicular from that point to the Earth's center will
// form a right triangle.  If the length of this perpendicular is less than
// EARTH_RADIUS + ATMOS_MARGIN, the link cannot be established.
//
int SatGeometry::are_satellites_mutually_visible(coordinate first, coordinate second)
{
	// if we drop a perpendicular from the ISL to the Earth's surface,
	// we have a right triangle.  The atmospheric margin is the minimum
	// ISL grazing altitude.
	double c, d, min_radius, grazing_radius;
	double radius = get_radius(first); // could just use first.r here.
	double distance_ = distance(first, second);
	c = radius * radius;
	d = (distance_/2) * (distance_/2);
	grazing_radius = (EARTH_RADIUS + ATMOS_MARGIN);
	min_radius = sqrt(c - d);
	if (min_radius >= grazing_radius) {
		return 1;
	} else {
		return 0;
	}
}



SatPosition::SatPosition()    
{
}


/////////////////////////////////////////////////////////////////////
// class PolarSatPosition
/////////////////////////////////////////////////////////////////////

PolarSatPosition::PolarSatPosition(double altitude, double Inc, double Lon, 
    double Alpha, double Plane) : next_(0), plane_(0) {
	set(altitude, Lon, Alpha, Inc);
        if (Plane) 
		plane_ = int(Plane);
	type_ = POSITION_SAT_POLAR;
}

PolarSatPosition::PolarSatPosition(double altitude, double Inc, double Lon, 
    double Alpha, int Plane, int index) {
    set(altitude, Lon, Alpha, Inc);
    plane_ = Plane;
    index_ = index;
    type_ = POSITION_SAT_POLAR;
}

//
// Since it is easier to compute instantaneous orbit position based on a
// coordinate system centered on the orbit itself, we keep initial coordinates
// specified in terms of the satellite orbit, and convert to true spherical 
// coordinates in coord().
// Initial satellite position is specified as follows:
// initial_.theta specifies initial angle with respect to "ascending node"
// i.e., zero is at equator (ascending)-- this is the $alpha parameter in Otcl
// initial_.phi specifies East longitude of "ascending node"  
// -- this is the $lon parameter in OTcl
// Note that with this system, only theta changes with time
//
void PolarSatPosition::set(double Altitude, double Lon, double Alpha, double Incl)
{
	if (Altitude < 0) {
		fprintf(stderr, "PolarSatPosition:  altitude out of \
		   bounds: %f\n", Altitude);
		exit(1);
	}
	initial_.r = Altitude + EARTH_RADIUS; // Altitude in km above the earth
	if (Alpha < 0 || Alpha >= 360) {
		fprintf(stderr, "PolarSatPosition:  alpha out of bounds: %f\n", 
		    Alpha);
		exit(1);
	}
	initial_.theta = DEG_TO_RAD(Alpha);

	if (Lon < -180 || Lon > 180) {
		fprintf(stderr, "PolarSatPosition:  lon out of bounds: %f\n", 
		    Lon);
		exit(1);
	}
	if (Lon < 0)
		initial_.phi = DEG_TO_RAD(360 + Lon);
	else
		initial_.phi = DEG_TO_RAD(Lon);

	if (Incl < 0 || Incl > 180) {
		// retrograde orbits = (90 < Inclination < 180)
		fprintf(stderr, "PolarSatPosition:  inclination out of \
		    bounds: %f\n", Incl); 
		exit(1);
	}
	inclination_ = DEG_TO_RAD(Incl);
	// XXX: can't use "num = pow(initial_.r,3)" here because of linux lib
	double num = initial_.r * initial_.r * initial_.r;
	period_ = 2 * PI * sqrt(num/MU); // seconds
    //cout<<"period_ of the constellation is: "<<period_<<endl;
}


//
// The initial coordinate has the following properties:
// theta: 0 < theta < 2 * PI (essentially, this specifies initial position)  
// phi:  0 < phi < 2 * PI (longitude of ascending node)
// Return a coordinate with the following properties (i.e. convert to a true
// spherical coordinate):
// theta:  0 < theta < PI
// phi:  0 < phi < 2 * PI
//
coordinate PolarSatPosition::coord(int NOW)
{
	coordinate current;
	double partial;  // fraction of orbit period completed
	partial = 
	    (fmod(NOW, period_)/period_) * 2*PI; //rad
	double theta_cur, phi_cur, theta_new, phi_new;

	// Compute current orbit-centric coordinates:
	// theta_cur adds effects of time (orbital rotation) to initial_.theta
	theta_cur = fmod(initial_.theta + partial, 2*PI);
	phi_cur = initial_.phi;
	// Reminder:  theta_cur and phi_cur are temporal translations of 
	// initial parameters and are NOT true spherical coordinates.
	//
	// Now generate actual spherical coordinates,
	// with 0 < theta_new < PI and 0 < phi_new < 360

	// asin returns value between -PI/2 and PI/2, so 
	// theta_new guaranteed to be between 0 and PI
	theta_new = PI/2 - asin(sin(inclination_) * sin(theta_cur));
	// if theta_new is between PI/2 and 3*PI/2, must correct
	// for return value of atan()
	if (theta_cur > PI/2 && theta_cur < 3*PI/2)
		phi_new = atan(cos(inclination_) * tan(theta_cur)) + 
			phi_cur + PI;
	else
		phi_new = atan(cos(inclination_) * tan(theta_cur)) + 
			phi_cur;
	phi_new = fmod(phi_new + 2*PI, 2*PI);
	
	current.r = initial_.r;
	current.theta = theta_new;
	current.phi = phi_new;
	return current;
}


//
// added by Lloyd Wood, 27 March 2000.
// allows us to distinguish between satellites that are ascending (heading north)
// and descending (heading south).
//
bool PolarSatPosition::isascending(int NOW)
{	
	double partial = (fmod(NOW, period_)/period_) * 2*PI; //rad
	double theta_cur = fmod(initial_.theta + partial, 2*PI);
	if ((theta_cur > PI/2)&&(theta_cur < 3*PI/2)) {
		return 0;
	} else {
		return 1;
	}
}

 
