int enA = 9;
int in1 = 8;
int in2 = 7;

int avgSpeed;
int sensorPin;
float Referencia = 20;
//GANANCIAS
float Kp = 16;
float Ki = 0.1;
float Kd = 0.15;

double u;
double absPID;
double PID_PWM;

double u_prev;
double error;

void setup() {
  // Comunicación seria a 9600 baudios
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}


void loop() {
  
  long tiempo=millis(); //tiempo antes de iniciar la lectura
  int D_cm=distancia(25); //lectura de distancia
  tiempo=millis()-tiempo; //milisegundos que duró la lectura
  //Serial.print("Tiempo de lectura: ");
  Serial.print(tiempo); 
  Serial.print("ms  Distancia: ");
  Serial.print(D_cm);
  Serial.println("  cm");
  Serial.print("error: ");
  Serial.print(error);
  Serial.println(" e");
  delay(100);

  double T = 1;
  u = PID(20,D_cm,Kd,Ki,Kp,u_prev,T);
  u_prev = u;

  normalizeData(u);

  absPID = abs (u);
  PID_PWM = map(absPID, 0, 256, 0, 255);

  DirectionRotation(u,PID_PWM);
}

float distancia(int n)
{
  long suma=0;
  for(int i=0;i<n;i++)
  {
    suma=suma+analogRead(A0);
  }  
  float adc=suma/n;
  float distancia_cm = 9608.2851 * pow(adc, -1.1051);
  return(distancia_cm);
}

int PID(int setPoint,int sensorValue,float kd,float ki, float kp, double u_prev,double T){ 
  double u;
  double A;
  double B;
  double prev_error = 0.0;
  double prev_error2;

  double Errors[3] = {error,prev_error,prev_error2};

  error = setPoint - sensorValue;
  Serial.print(error);

  A = (1.0/T)*(kp*T + ki*T +kd) ;
  B = (1.0/T)*(-kp*T-2.0*kd);
  C = (1.0/T)*kd;

  u = A*Errros[0] + B*Errors[1] + C*Errors[2] + u_prev;
  
  prev_error2 = prev_error;
  prev_error = error;
  

  return u;

}

int DirectionRotation(double u,int PWM){
  if(u < 0){
    digitalWrite(in1,0);
    digitalWrite(in2,1);
    analogWrite(enA, PWM);

  }
  if(u >= 0){
    digitalWrite(in1,1);
    digitalWrite(in2,0);
    analogWrite(enA, PWM);
  }

}

int normalizeData(double u){
  if(u > 255){
    u = 255;
  }
  if(u > -255){
    u = -255;
  }
}