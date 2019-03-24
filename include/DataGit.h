#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99
#define WAYPOINT_DIST_TOLERANE 5
#define HEADING_TOLERANCE 5

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

#define Task_t 10          // Task Time in milli seconds
double destinationlat = 0;
double destinationlon = 0; 
int headingError;
int sat = 0;
float kmh = 0;
long olddistance = 0;
 double lat = 0;
double lon = 0;
int dt=0;
unsigned long t;

enum directions
{
  left = TURN_LEFT,
  right = TURN_RIGHT,
  straight = TURN_STRAIGHT
};
directions turnDirection = straight;


struct geoloc {
  float lat;
  float lon;
};

struct geoloc way1;
struct geoloc way2;
struct geoloc way3;
struct geoloc way4;
struct geoloc way5; 
void initializeData(){
way1.lat = *********;
way1.lon = *********;
way2.lat = *********;
way2.lon = *********;
way3.lat = *********;
way3.lon = *********;
way4.lat = *********;
way4.lon = *********;
way5.lat = *********; 
way5.lon = *********;
}  