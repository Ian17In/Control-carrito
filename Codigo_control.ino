/* Ejercicio de medicion del sensor de distancia*/
/*Con promedio y distancia*/

// Motor A
 
int enA = 9;
int in1 = 8;
int in2 = 7;


// Motor B
 
int enB = 3;
int in3 = 5;
int in4 = 4;



void setup() {
  // Comunicación serial a 9600 bits/s
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(6,HIGH);

}

double lastTime = 0;
float errIntegral = 0;
float dErr = 0;
float eant = 0;
int PID;
int absPID;
int PID_PWM;

//REFERENCIA
float Referencia = 20;

//GANANCIAS
float Kp = 20;
float Ki = (0.1)*1.3;
float Kd = (0.15)*1.08;


void loop() {
  
  long tiempo=millis(); //tiempo antes de iniciar la lectura
  float D_cm=distancia(25); //lectura de distancia
  tiempo=millis()-tiempo; //milisegundos que duró la lectura
  Serial.print("Tiempo de lectura: ");
  Serial.print(tiempo); 
  Serial.print("ms  Distancia: ");

  Serial.print(D_cm);
  Serial.println("  cm");
  

  //MEDICION DE LA DISTANCIA
  //D_cm=distancia(10); 

  //CALCULO DEL ERROR
  float error = Referencia - D_cm;
  Serial.println("Error: ");
  Serial.println(error);

  //DEFINICIÓN DEL TIEMPO
  unsigned long now = millis();
  double timeChange = (double) (now - lastTime);

  //CALCULO DE LA INTEGRAL DEL ERROR
  errIntegral += (error * (timeChange * 0.001));

  //CALCULO DE LA DERIVADA DEL ERROR
  dErr = (error - eant) / (timeChange);

  //CALCULO DEL PID
  PID = (Kp * error) + (Ki * errIntegral) + (Kd * dErr);

  //SATURACION DEL PID
  if (PID > 255) {
    PID = 255;
  }
  if (PID < - 255) {
    PID = - 255;
  }

  //MAGNITUD DEL PID
  absPID = abs (PID);

  //MAPEO DEL VALOR DEL PID A SEÑAL DE PWM DE 0 a 255
  PID_PWM = map(absPID, 0, 256, 0, 255);

  //SIGNO DEL PID PARA SENTIDO DE GIRO

  if (PID < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, PID_PWM);
    //digitalWrite(in3, LOW);
    //digitalWrite(in4, HIGH);
    //analogWrite(enB, 0.97*PID_PWM);
  }

  if (PID >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, PID_PWM);
    //digitalWrite(in3, HIGH);
    //digitalWrite(in4, LOW);
    //analogWrite(enB, 0.97*PID_PWM);
  }

  //MEMORIA DE VARIABLES
  eant = error;
  lastTime = now;


  
  delay(7);
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