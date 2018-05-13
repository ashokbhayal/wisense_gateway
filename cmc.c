#include <stdio.h>



int main(int argc, char **argv)
{
   int w1, w2;
   int adcW1, adcW2;
   int mFactor, cFactor;

   printf("\nEnter value of weight 1 in grams : ");
   scanf("%d", &w1);

   if (w1 == 0)
   {
       printf("\nPlease enter valid weight 1 !! \n");
       return 1;
   }

   printf("\nEnter value of weight 2 in grams : ");
   scanf("%d", &w2);

   if (w2 == 0)
   {
       printf("\nPlease enter valid weight 2 !! \n");
       return 1;
   }

   if (w1 == w2)
   {
       printf("\nYou have entered the same value for weight 1 and weight 2 !! \n");
       return 1;
   }


   printf("\nEnter average adc output corresponding to weight 1 : ");
   scanf("%d", &adcW1);

   if (adcW1 == 0)
   {
       printf("\nEnter valid adc output corresponding to weight 1 !! \n");
       return 1;
   }
   
   printf("\nEnter average adc output corresponding to weight 2 : ");
   scanf("%d", &adcW2);

   if (adcW2 == 0)
   {
       printf("\nEnter valid adc output corresponding to weight 2 !! \n");
       return 1;
   }

   mFactor = ((adcW1 - adcW2) * 1000) / (w1 - w2);


   cFactor = (adcW1 - ((mFactor * w1)/1000)); 

   printf("\nLoad cell equation is :  f(W) = %d*(W) %c %d  \n",
          mFactor, cFactor > 0 ? '+' : '-', cFactor < 0 ? -cFactor : cFactor);
  
   return 0; 
}
