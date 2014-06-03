#include "example.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
Platform* p;
float getRand(){
    int value=rand()%4000-2000;
    return value/100.0;
}

int zapis(float array[]){
    char ret=0;
    int c;
    while(ret==0){
        c=getchar();
        if(c=='z'){
            array[0]=0;
            array[1]=0;
            array[2]=0;
            array[3]=0;
            array[4]=0;
            array[5]=0;
             int resp=p->setPositions(array);
             printf("val %d\n",resp);
        }else{
            ret=1;
            int resp=p->setPositions(array);
            printf("val %d\n",resp);
        }
    }
    float arr[]={0,0,0,0,0,0};
    p->getPositions(arr);
    printf("%f %f %f %f %f %f \n",arr[0],arr[1],arr[2],arr[3],arr[4],arr[5]);


    return c;
}
int main(){
    //
    p= new Platform(3);
    float array[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    p->setPositions(array);
    Sleep(1000);
    float current_val=25;
    short cnt=0;
    int znak=0;
    if(true){
        while(znak!='n'){
            cnt++;
            if(cnt%2==1){
                array[0]=-current_val;
                znak=zapis(array);
                printf("x=%f\n",current_val);
            }else{
                array[0]=current_val;
                znak=zapis(array);
                printf("x=-%f\n",current_val);
            }
        }
        znak=0;
        cnt=0;
        while(znak!='n'){
            cnt++;
            if(cnt%2==1){
                array[0]=0.0;
                array[1]=-current_val;
                znak=zapis(array);
                printf("y=-%f\n",current_val);
            }else{
                array[1]=current_val;
                znak=zapis(array);
                printf("y=%f\n",current_val);
            }
        }
        znak=0;
        cnt=0;
        while(znak!='n'){
            cnt++;
            if(cnt%2==1){
                array[1]=0.0;
                array[2]=-current_val;
                znak=zapis(array);
                printf("z=-%f\n",current_val);
            }else{
                array[2]=current_val;
                znak=zapis(array);
                printf("z=%f\n",current_val);
            }
        }
        znak=0;
        cnt=0;
        while(znak!='n'){
            cnt++;
            if(cnt%2==1){
                array[2]=0.0;
                array[3]=current_val;
                znak=zapis(array);
                printf("rot(x)=%f°\n",current_val);
            }else{
                array[3]=-current_val;
                znak=zapis(array);
                printf("rot(x)=-%f°\n",current_val);
            }
        }
        znak=0;
        cnt=0;
        while(znak!='n'){
            cnt++;
            if(cnt%2==1){
                array[3]=0.0;
                array[4]=current_val;
                znak=zapis(array);
                printf("rot(y)=%f°\n",current_val);
            }else{
                array[4]=-current_val;
                znak=zapis(array);
                printf("rot(y)=-%f°\n",current_val);
            }
        }
        cnt=0;
        znak=0;
        while(znak!='n'){
            cnt++;
            if(cnt%2==1){
                array[4]=0.0;
                array[5]=current_val;
                znak=zapis(array);
                printf("rot(z)=%f°\n",current_val);
            }else{
                array[5]=-current_val;
                znak=zapis(array);
                printf("rot(z)=-%f°\n",current_val);
            }
        }
    }else{
        for(int i=0;i<1000;i++){
            for(int k=0;k<6;k++){
                array[k]=getRand();
            }
            p->setPositions(array);
            Sleep(3000);
        }
    }
    p->endCommunication();
    delete p;
    return 1;
}
