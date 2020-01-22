#define LEFT_MOTOR_PIN1 
#define LEFT_MOTOR_PIN2
#define LEFT_MOTOR_SPEED_PIN

#define RIGHT_MOTOR_PIN1
#define RIGHT_MOTOR_PIN2
#define RIGHT_MOTOR_SPEED_PIN

#define MODE_SWITCH_PIN

#define SIDE_TURN_TIME 
#define U_TURN_TIME


#define KP
#define KD
#define KI
#define midpointValue


#define MODE_EXPLORE 0
#define MODE_SHORTEST 1


int rawLineDigital = [0,0,0,0,0,0,0,0];

string lineInputPins = ["A1", "A2", "A3", "A4", "A5", "2", "3"];

class planner
{
    short error_mem[50];
    long dCounter;
    float kP, kD, kI;
    int mode;

    void initPlanner()
    {
        dCounter = 0;
        for(int n = 0 ; n < 50 ; n++)
        {
            error_mem[n]=0;
        }
        kP = float(KP);
        kD = float(KD);
        kI = float(KI);
        mode = MODE_EXPLORE;

    }
    int basic_detect_line()
    {
        int value = -1;
        int sum = 0;
        int num_active = 0;
        for(int i = 0; i < rawLineDigital.size(); i++)
        {
            rawLineDigital[i] = digitalRead(lineInputPins);
            sum += (10*i)*rawLineDigital[i];
            num_active++;
        }
        if(num_active == 0)
        {
            return value;
        }
        value = sum / num_active;
        return value;
    }

    int advanced_detect_line()
    {
        //decide a metric and define it. Do we want to check it over time?
    }

    int detect_junction()
    {
        midpoint = rawLineDigital.size()/2;
        int leftval = 0, rightval = 0, centerval = 0;
        //checking left turning junction
        for(int i = 0; i<=midpoint; i++)
        {
            leftval += (10*(midpoint-i)*rawLineDigital[i]);
            rightval += (10*(i-midpoint)*rawLineDigital[i]);
            //reevaluate these again and compute centerval

        }

        //need to have at least a minimum number of IR pairs at digital high or determining turn.

        if (leftval > SIDE_TURN_THRESHOLD)
        {
            //add to graph and tree for left turn
        }
        if (rightval > SIDE_TURN_THRESHOLD)
        {
            //add to graph and tree for right turn
        }
        if (MIN_CENTER_THRESHOLD < centerval && centerval < MAX_CENTER_THRESHOLD)
        {
            //add to graph // at this time, the right and left need to be zero

        }


    }

    void calculate_motor_speeds(int sensor_val)
    {
        int lineval = basic_detect_line();
        
        int error = midval - lineval;
        

        //use PID here

    }

    void set_global_direction(int left_mot_speed, int right_mot_speed)
    {
        if(right_mot_speed >= 0)
        {
            set_direction(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, 1);
        }
        else
        {
            set_direction(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, 0);
        }


        if(left_mot_speed >= 0)
        {
            set_direction(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, 1);
        }
        else
        {
            set_direction(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, 0);
        }
    }

    void apply_motor_speeds(int left_mot_speed, int right_mot_speed)
    {
        analogWrite(RIGHT_MOTOR_SPEED_PIN, right_mot_speed);
        analogWrite(LEFT_MOTOR_SPEED_PIN, left_mot_speed);
    }

    int detect_end()
    {
        //use buffer to check if over time if (most part of) the sensor gives positive.
    }

    void set_direction(int pin1, int pin2, int dir)
    {
        digitalWrite(pin1, dir);
        digitalWrite(pin2, (!dir));
    }

    void hard_left()
    {
        set_direction(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, 0);
        set_direction(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, 1);
        apply_motor_speeds(TURN_SPEED, TURN_SPEED);
        delay(SIDE_TURN_TIME);
        set_direction(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, 1);
        set_direction(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, 1);
    }

    void hard_right()
    {
        set_direction(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, 1);
        set_direction(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, 0);
        apply_motor_speeds(TURN_SPEED, TURN_SPEED);
        delay(SIDE_TURN_TIME);
        set_direction(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, 1);
        set_direction(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, 1);
    }

    void hard_u_turn()
    {
        set_direction(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, 1);
        set_direction(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, 0);
        apply_motor_speeds(TURN_SPEED, TURN_SPEED);
        delay(U_TURN_TIME);
        set_direction(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, 1);
        set_direction(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, 1);
    }
    
}



void setup()
{
    
}

void loop()
{

}