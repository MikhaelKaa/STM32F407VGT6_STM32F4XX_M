


int main(void)
{

  while (1)
  {
    for(volatile int i = 0; i < 1000;) {
      i++;
    }
  }

}

void SystemInit (void)
{
  //SystemCoreClock = SYSTEM_CLOCK;
}