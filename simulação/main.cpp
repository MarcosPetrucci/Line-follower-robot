// Habilite o server antes na simulação V-REP com o comando lua:
// simExtRemoteApiStart(portNumber) -- inicia servidor remoteAPI do V-REP

extern "C"
{
#include "remoteApi/extApi.h"
}

#include <iostream>
#include <string.h>

using namespace std;

int main(int argc, char **argv)
{
  //Variaveis para conexao do servidor
  string serverIP = "127.0.0.1";
  int serverPort = 19999;
  //Handles dos  motores
  int leftMotorHandle = 0;
  float vLeft = 0;
  int rightMotorHandle = 0;
  float vRight = 0;
  //Handles e nomes dos sensores
  string sensorNome[5] = {"LeftSensor", "MiddleSensor", "RightSensor", "LeftSensor2", "RightSensor2"};
  int sensorHandle[5];

  int res[2];
  simxUChar* image;
  int sensorResponse[5] = {0,0,0,0,0};

  //Tenta estabelecer conexao com a simulacao (nao esqueca de dar play)
  int clientID = simxStart((simxChar *)serverIP.c_str(), serverPort, true, true, 2000, 5);

  //Se a conexao e estabelicida, sera retornado um valo diferente de 0
  if (clientID != -1)
  {
    printf("Servidor conectado!\n");
    
    // inicialização dos motores
    if (simxGetObjectHandle(clientID, (const simxChar *)"DynamicLeftJoint", (simxInt *)&leftMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      printf("Handle do motor esquerdo nao encontrado!\n");
    else
      printf("Conectado ao motor esquerdo!\n");

    if (simxGetObjectHandle(clientID, (const simxChar *)"DynamicRightJoint", (simxInt *)&rightMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      printf("Handle do motor direito nao encontrado!\n");
    else
      printf("Conectado ao motor direito!\n");

    
    // inicialização dos sensores (remoteApi)
    for (int i = 0; i < 5; i++)
    {
      if (simxGetObjectHandle(clientID, (const simxChar *)sensorNome[i].c_str(), (simxInt *)&sensorHandle[i], (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      {
        printf("Handle do sensor %s nao encontrado!\n",sensorNome[i].c_str());
      }
      else
      {
        printf("Conectado ao sensor %s \n",sensorNome[i].c_str());
        simxGetVisionSensorImage(clientID,sensorHandle[i],res,&image,0,simx_opmode_streaming);
      }
    }
  
    // desvio e velocidade do robô
    while (simxGetConnectionId(clientID) != -1) // enquanto a simulação estiver ativa
    {
      for (int i = 0; i < 5; i++)
      {
        if(simxGetVisionSensorImage(clientID,sensorHandle[i],res,&image,0,simx_opmode_streaming) == simx_return_ok)
        {
          int sum = 0;
          for (int i = 0; i < res[0]*res[0]; i++)
          {
            sum += (int) image[i];
          }
          sum = sum/(res[0]*res[0]); //Média dos valores preto-branco dos pixels da imagem
                                     //Podemos simular esse valor como um sensor IR
          if(sum > 60)
            sensorResponse[i] = 1;
          else
            sensorResponse[i] = 0;
          
          printf("Sensor[%i] : %d \n",i,sensorResponse[i]);
        }
      }
      char aux;
      scanf("%c",&aux);
      if(aux == 'w')
      {
        vLeft = 2;
        vRight = 2;
      }
      else if (aux == 'd')
      {
        vLeft = 0;
        vRight = 2;
      }
      else if (aux == 'a')
      {
        vLeft = 2;
        vRight = 0;
      }
      else if (aux == 's')
      {
        vLeft = -2;
        vRight = -2;
      }
      
      // atualiza velocidades dos motores
      simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)vLeft, simx_opmode_streaming);
      simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)vRight, simx_opmode_streaming);
      
      extApi_sleepMs(5);
    }
    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexao fechada!" << std::endl;
  }
  else
  {
    printf("Problemas para conectar o servidor!\n");
  }
  return 0;
}
