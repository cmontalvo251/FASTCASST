//These are standard C-libraries like numpy or scipy
#include <unistd.h>
#include <cstdio>

//these are modules created by emlid
//Keep in mind that these two are in here. In case you are using a Navio1 since
//we are using a Navio2 I'm just gonna comment the second one out
#include <Navio2/RCInput_Navio2.h> //Head over to this header for more comments
//#include <Navio+/RCInput_Navio.h>


#include <Common/Util.h>
#include <memory>

#define READ_FAILED -1

std::unique_ptr <RCInput> get_rcin()
{
  //if (get_navio_version() == NAVIO2)
  //{

  //SAME THING DOWN HERE. Btdubs that we are only using Navio2 for this project.
  //So. it means that we don't need an else statement. Honestly we could probably just remove
  //this function and hardcode these lines in the main. but this will do for now.
        auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
	printf("Hey you are using a NAVIO2 TWO!!!!!!!!!\n");
        return ptr;
	//    } else
	//    {
	//        auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio() };
	//	printf("Why are you using a Navio ONE?????? Upgrade your shit. \n");
	//        return ptr;
	//    }

}


int main(int argc, char *argv[])
{
  //RCInput rcin{}; //what? Why are you declaring this variable twice??? Seems like we need to chat....
    
    if (check_apm()) {
        return 1;
    }

    auto rcin = get_rcin(); //See this code is using something super fucking fancy where instead of saying the variable is a double or of class RCInput they are telling the compiler that this is an auto variable. So
    //the compiler with "auto"matically figure out what type of variable this is.

    //This initialize routine will loop through NUM_CHANNELS so make sure to change NUM_CHANNELS
    //in RCINput_Navio2.h
    rcin->initialize();

    //How many channels do you want?
    //This #define NUM_CHANNELS is created in RCInput_Navio2.h
    rcin->channel_count = NUM_CHANNELS;
    printf("There are %d channels \n",rcin->channel_count);
    
    while (true)
    {
      for(int i = 0; i < rcin->channel_count ; i++)
      {
      int period = rcin->read(i);
        printf("%d ", period);
      }
      printf("\n"); 
      sleep(0.1);
      //printf("hey \n");
    }

    return 0;
}

