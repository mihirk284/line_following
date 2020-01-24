#include<String.h>
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


#define MOTOR_BASE_SPEED


#define MODE_EXPLORE 0
#define MODE_SHORTEST 1

#define LEFT "L"
#define RIGHT "R"
#define STRAIGHT "S"
#define BACK "B"




int lineInputPins = [A0, A1, A2, A3, A4, A5, A6, A7];

class planner
{
    public:

    short errorArray[50];
    long dCounter;
    float kP, kD, kI;
    int mode;
    long long global_reading_counter;
    int motor_base_speed;
    String raw_path = "";
    int midval, num_active, prev_num_active;
    int rawLineDigital[8];
    int prev_mem[8];

    void initPlanner()
    {
        dCounter = 0;
        for(int n = 0 ; n < 50 ; n++)
        {
            errorArray[n]=0;
        }
        kP = float(KP);
        kD = float(KD);
        kI = float(KI);
        mode = int(MODE_EXPLORE);
        motor_base_speed = int(MOTOR_BASE_SPEED);
        global_reading_counter = 0;
        midval = 35;
        rawLineDigital = [0,0,0,0,0,0,0,0];
        prev_mem = [0, 0, 0, 0, 0, 0, 0, 0];
        prev_num_active = 0;
        num_active = 0;

    }

    int basic_detect_line()
    {
        int value = -1;
        int sum = 0;
        num_active = 0;

        //reading pins from the sensor
        for(int i = 0; i < rawLineDigital.size(); i++)
        {
            int reading = digitalRead(lineInputPins);
            rawLineDigital[i] = reading;
            sum += (10*i)*rawLineDigital[i];
            if (reading == 1)
            {
                num_active++;
            }
            Serial.println(rawLineDigital);
        }
        if(num_active == 0)
        {
            return value;
        }
        value = sum / num_active;
        return value;
    }

    // int advanced_detect_line()
    // {
    //     //decide a metric and define it. Do we want to check it over time?
    // }

    // int detect_junction()
    // {
    //     float midpoint = (rawLineDigital.size()-1)/2.0;
    //     float leftval = 0, rightval = 0, centerval = 0;
    //     //checking left turning junction
    //     for(int i = 0; i<=midpoint; i++)
    //     {
    //         leftval += (10*(midpoint-i)*rawLineDigital[i]);
    //         rightval += (10*(i-midpoint)*rawLineDigital[i]);
    //         //reevaluate these again and compute centerval
    //     }

    //     //need to have at least a minimum number of IR pairs at digital high or determining turn.

    //     if (leftval > SIDE_TURN_THRESHOLD)
    //     {
    //         //add to graph and tree for left turn
    //     }
    //     if (rightval > SIDE_TURN_THRESHOLD)
    //     {
    //         //add to graph and tree for right turn
    //     }
    //     if (MIN_CENTER_THRESHOLD < centerval && centerval < MAX_CENTER_THRESHOLD)
    //     {
    //         //add to graph // at this time, the right and left need to be zero

    //     }
    // }


    int basic_detect_junction()
    {
        int junction_val = -1;
        if(rawLineDigital[0] == 1 )
        {
            junction_val = 0;
        }
        else if (rawLineDigital[3] == 1 || rawLineDigital[4] == 1 || rawLineDigital[5] == 1 || rawLineDigital[2] == 1)
        {
            junction_val =1;
        }
        else if (rawLineDigital[7] == 1)
        {
            junction_val = 2;
        }
        
        
        return junction_val;
    }


    void junction_handler(int turn)
    {
        if (turn == -1)
        {
            return;
        }
        if(turn == 0)
        {
            raw_path.concat(LEFT);
            hard_left();
        }
        else if(turn == 1)
        {
            raw_path.concat(STRAIGHT);
            follow_line();
        }
        else if(turn == 2)
        {
            raw_path.concat(RIGHT);
            hard_right();
            
        }
        else if(turn == 3)
        {
            raw_path.concat(BACK);
            hard_u_turn();
        }
        return;
    }

    void follow_line(int sensor_val)
    {
        int error = sensor_val - midval;
        errorArray[global_reading_counter%50] = error;
        global_reading_counter++;
        long errSum = 0;
        for(int i = 0; i< 50; i++)
        {
            errSum += errorArray[i];
        }
        int integrated_error = errSum;
        
        int speed = kP*(error) - kD(error - prev_error) + kI*(integrated_error);

        int left_mot_speed = MOTOR_BASE_SPEED + speed;
        int right_mot_speed = MOTOR_BASE_SPEED - speed;

        prev_error = error;
        apply_motor_speeds(left_mot_speed, right_mot_speed);
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
        if (left_mot_speed > 255)
        {
            left_mot_speed = 255;
        }
        else if (left_mot_speed < -255)
        {
            left_mot_speed = -255;
        }


        if(right_mot_speed > 255)
        {
            right_mot_speed = 255;
        }
        else if (right_mot_speed < -255)
        {
            right_mot_speed = -255;
        }
        
        set_global_direction(left_mot_speed, right_mot_speed);

        left_mot_speed = abs(left_mot_speed);
        right_mot_speed = abs(right_mot_speed);
        analogWrite(RIGHT_MOTOR_SPEED_PIN, right_mot_speed);
        analogWrite(LEFT_MOTOR_SPEED_PIN, left_mot_speed);
    }

    int detect_end()
    {
        if(num_active >= 6 && prev_num_active >=6 && global_reading_counter%5 > 2)
        {
            Serial.println("ENDPOINT REACHED");
            return 1;
        }
        else
        {
            return 0;
        }
        
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


    void add_to_memory()
    {

        if(global_reading_counter %5 == 0)
        {
            for(int i = 0; i < 8; i++)
            {
                prev_mem[i] = rawLineDigital[i];
            }
        }
        prev_num_active = num_active;      
    }

    void end_run()
    {
        apply_motor_speeds(0,0);
    }


    void decision_maker()
    {
        int detected_val = basic_detect_line();
        if (num_active > 5)
        {
            int jn_type = basic_detect_junction();
            if(jn_type == -1)
            {
                follow_line();
            }
            else
            {
                junction_handler(jn_type);
            }
            if(detect_end() ==1)
            {
                end_run();
            }          
            
        }

    }
    
}



void setup()
{
    
}

void loop()
{

}