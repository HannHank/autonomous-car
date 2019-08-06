//this header initialize all of the ultrasonic_sensors
const int tri_ultrasinic_FL = 40;
const int echo_ultrasinic_FL = 41;
const int tri_ultrasinic_FR = 42; 
const int echo_ultrasinic_FR = 43;


void define_Ultrasonic_pins(){
    pinMode(tri_ultrasinic_FL,OUTPUT);
    pinMode(echo_ultrasinic_FL,INPUT);

    pinMode(tri_ultrasinic_FR,OUTPUT);
    pinMode(echo_ultrasinic_FR,INPUT);
}