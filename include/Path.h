#include <stdlib.h>
#include <Arduino.h>

double Path_lat = 0;
double Path_lon = 0; 

struct Node* ptr;

using namespace std;
struct Node{
    double lat;
    double lon;
    struct Node *next;
};

struct Node* head = 0; 
void insert(double lat, double lon){
    struct Node* new_node = (struct Node*) malloc(sizeof(struct Node));
    new_node->lat = lat;
    new_node->lon = lon;
    new_node->next = head;
    head = new_node;
}

void deletList(){
    struct Node *temp;
    while(head != NULL){
        temp = head;
        head = head->next;
        free(temp);
    }
    
}
void displayPath() { 
   struct Node* ptr;
   //cout<< ptr->data;
   ptr = head;
   while (ptr != NULL) { 
      //cout<< ptr->data;
      Serial.print("lat: ");
      Serial.print(ptr->lat);
      Serial.print("lon: ");
      Serial.print(ptr->lon);
      

      ptr = ptr->next; 
   } 
   
} 

void initializePath(){
     //hier habe ich das struct Node* ptr; weggenommen
     //Path wird auf das erste Node gesetzt
     ptr = head;
}

