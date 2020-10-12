
#define ELA P10_5

int ELAticks = 0;

int ELA_maxticks = 10; //tune this 

int ELA_start_time = 0;
bool ELA_last_state = 0;
bool ELA_current_state = 0;

void setup2 ()
{
  pinMode(ELA, INPUT);
  
  ELAticks = ELA_maxticks;
}

void loop2 ()
{
  ELA_start_time = millis();
  ELA_last_state = 0;
  ELA_current_state = 0;
  
 while (ELAticks < ELA_maxticks)
  {
    ELA_current_state = digitalRead(ELA);
    
    if (ELA_current_state && !ELA_last_state) //rising edge
    {
      ELAticks++;
    }
    
    ELA_last_state = ELA_current_state;
  }
  
  diffLA = millis() - ELA_start_time; //difference in time 
  
  ELAticks = 0;
}
