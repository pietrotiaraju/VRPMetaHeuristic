#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <chrono>
#include <random>
#include <tr1/random>
#define DIM 50
#define SPACE 100
#define NODES 101
#define uBestEqualsCurrent 0
#define uCurrentEqualsBest 1
#define uDisturbanceEqualsCurrent 2
#define uCurrentEqualsDisturbance 3
#define uDisturbanceEqualsBest 4

//Definindo o tipo rota
typedef struct Route
{
    int maxCapacity;
    int ID[NODES], coordX[NODES], coordY[NODES], demand[NODES], inertia[NODES], binaryAllocation[NODES];
    int neighborhoodControl;
    double prob[NODES], cumProb[NODES];
    int distanceMatrix[NODES][NODES];
    int disturbanceSolution[DIM][SPACE];
    int currentSolution[DIM][SPACE];
    int bestSolution[DIM][SPACE];
} route;

//Bool list
bool checkFeasibility(int capacity, int demand[NODES], int solution[DIM][SPACE]);

//Double list
double randomReal(int genMin, int genMax);

//Int list
int calculateResult(int dist[NODES][NODES], int solution[DIM][SPACE]);
int checkBinaryAllocation(int binary[NODES]);
int generateInitialNode(double random, double cumprob[NODES]);
int getNearestDistance(int id, int dist[NODES][NODES], int binary[NODES]);
int getNearestNode(int id, int dist[NODES][NODES], int binary[NODES]);
int getNodeIndex(int node, int route, int solution[DIM][SPACE]);
int hLength(int x, int matrix[DIM][SPACE]);
int randomInteger(int genMin, int genMax);
int vLength(int matrix[DIM][SPACE]);

//Void list
void calculateDistance(route *r);
void calculateInertia(route *r);
void calculateProb(route *r);
void cumulativeProb(route *r);
void displaySolution(int solution[DIM][SPACE]);
void dLeftShift(route *r);
void dInsert(route *r);
void dInterchange(route *r);
void dReallocation(route *r);
void dRightShift(route *r);
void dSwap(route *r);
void dTraditionalCross(route *r);
void emptyBinaryAllocation(route *r);
void emptyBestRoute(route *r);
void emptyCurrentRoute(route *r);
void emptyDisturbanceRoute(route *r);
void generateInitialSolution(route *r);
void improvedInitialSolution(int freq, route *r);
void mainControl(int depth, bool echo, route *r);
void mainDisturbance(double tol, int depth, route *r);
void mainNeighborhood(int depth, bool echo, route *r);
void hLeftShift(int depth, bool echo, route *r);
void hInsert(int depth, bool echo, route *r);
void hInterchange(int depth, bool echo, route *r);
void hReallocation(int depth, bool echo, route *r);
void hRightShift(int depth, bool echo, route *r);
void hSwap(int depth, bool echo, route *r);
void hTraditionalCross(int depth, bool echo, route *r);
void importData(char dir[SPACE], route *r);
void updateSolution(int type, route *r);

using namespace std;

main()
{
    system("color F3");
    cout << "BEM-VINDO A MINHA HEURISTICA VRP\n";
    system("pause");

    route *master = new route;
    char path[SPACE] = "C:\\Users\\pietro\\Downloads\\A-VRP\\Instancias\\X101k25_C.txt";
    bool echo = 0;
    int depth = 100;
    master->maxCapacity = 206;

    emptyBestRoute(master);
    emptyCurrentRoute(master);
    emptyDisturbanceRoute(master);
    importData(path, master);
    calculateDistance(master);

    //generateInitialSolution(master);
    improvedInitialSolution(10000, master);

    cout << "Initial solution: " << calculateResult(master->distanceMatrix, master->currentSolution) << "\n";

    mainControl(depth, echo, master);

    return(0);
}

//Geracao de numeros aleatorios
unsigned seed0 = chrono::system_clock::now().time_since_epoch().count();
unsigned seed1 = chrono::system_clock::now().time_since_epoch().count();

default_random_engine randomUniform(seed0);
default_random_engine randomInt(seed1);

double randomReal(int genMin, int genMax)
{
    double random;

    uniform_real_distribution <double> generatorU(genMin, genMax);
    random = generatorU(randomUniform);
    return(random);
}

int randomInteger(int genMin, int genMax)
{
    int random;

    uniform_int_distribution <int> generatorI(genMin, genMax);
    random = generatorI(randomInt);
    return(random);
}

void improvedInitialSolution(int freq, route *r)
{
    int i, valsol = 100000;

    for(i = 0; i <= freq; i++)
    {
        generateInitialSolution(r);

        if(calculateResult(r->distanceMatrix, r->currentSolution) <= valsol)
        {
            updateSolution(uBestEqualsCurrent, r);
            valsol = calculateResult(r->distanceMatrix, r->bestSolution);
        }

        emptyCurrentRoute(r);
    }

    updateSolution(uCurrentEqualsBest, r);
}

//Importa a instancia
void importData(char dir[SPACE], route *r)
{
    int i = 0;
    FILE *conn = fopen(dir, "r");

    while(!feof(conn))
    {
        fscanf(conn, "%d %d %d %d", &(r->ID[i]), &(r->coordX[i]), &(r->coordY[i]), &(r->demand[i]));
        i++;
    }
}

//Zera matriz
void emptyCurrentRoute(route *r)
{
    int i, j;

    for(i = 0; i <= DIM - 1; i++)
    {
        for(j = 0; j <= SPACE - 1; j++)
        {
            r->currentSolution[i][j] = 0;
        }
    }
}

//Zera matriz
void emptyDisturbanceRoute(route *r)
{
    int i, j;

    for(i = 0; i <= DIM - 1; i++)
    {
        for(j = 0; j <= SPACE - 1; j++)
        {
            r->disturbanceSolution[i][j] = 0;
        }
    }
}

//Zera matriz
void emptyBestRoute(route *r)
{
    int i, j;

    for(i = 0; i <= DIM - 1; i++)
    {
        for(j = 0; j <= SPACE - 1; j++)
        {
            r->bestSolution[i][j] = 0;
        }
    }
}



//Calcula o numero de rotas
int hLength(int x, int matrix[DIM][SPACE])
{
    int i;
    int sum = 0;

    for(i = 0; i <= SPACE - 1; i++)
    {
       if(matrix[x][i] != 0)
       {
           sum++;
       }
    }

    return(sum);
}

//Calcula o tamanho da rota
int vLength(int matrix[DIM][SPACE])
{
    int i;
    int sum = 0;

    for(i = 0; i <= DIM - 1; i++)
    {
       if(matrix[i][0] != 0)
       {
           sum++;
       }
    }

    return(sum);
}

//Calcula quantas posicoes alocadas no binary tem
int checkBinaryAllocation(int binary[NODES])
{
    int i;
    int sum = 0;

    for(i = 0; i <= NODES - 1; i++)
    {
        sum += binary[i];
    }

    return(sum);
}

void emptyBinaryAllocation(route *r)
{
    int i;

    for(i = 0; i <= NODES - 1; i++)
    {
        r->binaryAllocation[i] = 0;
    }
}

void calculateDistance(route *r)
{
    int i, j;

    for(i = 0; i <= NODES - 1; i++)
    {
        for(j = 0; j <= NODES - 1; j++)
        {
            r->distanceMatrix[i][j] = round(pow(pow(r->coordX[i] - r->coordX[j], 2) + pow(r->coordY[i] - r->coordY[j], 2), 0.5));
        }
    }
}

void calculateInertia(route *r)
{
    int i;

    //Nao ha inercia para o super source!!
    for(i = 0; i <= NODES - 1; i++)
    {
        //Verifica a disponibilidade do no
        if(r->binaryAllocation[i] == 0)
        {
            r->inertia[i] = (r->distanceMatrix[i][0])*(r->demand[i]);
        }
        else
        {
            r->inertia[i] = 0;
        }
    }
}

void calculateProb(route *r)
{
    int i;
    double sum = 0;

    //Acumula a inercia
    for(i = 0; i <= NODES - 1; i++)
    {
        sum += r->inertia[i];
    }

    //Razao entre a participacao da inercia no total
    for(i = 0; i <= NODES - 1; i++)
    {
        r->prob[i] = r->inertia[i]/sum;
    }
}

void cumulativeProb(route *r)
{
    int i;
    double result[NODES];

    partial_sum(r->prob, r->prob + NODES, result);

    for(i = 0; i <= NODES - 1; i++)
    {
        r->cumProb[i] = result[i];
    }
}

//Gera o no inicio a partir de um numero aleatorio e a distribuicao
int generateInitialNode(double random, double cumprob[NODES])
{
    int i, initialNode;

    for(i = 0; i <= NODES - 2; i++)
    {
        if(random >= cumprob[i] && random < cumprob[i+1])
        {
            initialNode = i + 2;
            return(initialNode);
        }
    }

    return(0);
}

int getNearestNode(int id, int dist[NODES][NODES], int binary[NODES])
{
    int i, minID, node = 0;

    minID = 1000000;

    if(checkBinaryAllocation(binary) <= NODES - 1)
    {
        for(i = 0; i <= NODES - 1; i++)
        {
            if(dist[id-1][i] < minID && dist[id-1][i] > 0 && binary[i] == 0)
            {
                minID = dist[id-1][i];
                node = i + 1;
            }
        }
    }
    else
    {
        node = 0;
    }

    return(node);
}

int getNearestDistance(int id, int dist[NODES][NODES], int binary[NODES])
{
    int i, minID;

    minID = 1000000;

    if(checkBinaryAllocation(binary) <= NODES - 1)
    {
        for(i = 0; i <= NODES - 1; i++)
        {
            if(dist[id-1][i] < minID && dist[id-1][i] > 0 && binary[i] == 0)
            {
                minID = dist[id-1][i];
            }
        }
    }

    return(minID);
}

void generateInitialSolution(route *r)
{
    //Seta os contadores para zero e inicia o procedimento de construcao
    int i = 0, j = 1, k;
    int randomNode = 0, nextNode, dist[2];
    int currentCapacity = r->maxCapacity;

    emptyBinaryAllocation(r);
    r->binaryAllocation[0] = 1;

    while(checkBinaryAllocation(r->binaryAllocation) <= NODES - 1)
    {
        //Atualiza os dados
        calculateInertia(r);
        calculateProb(r);
        cumulativeProb(r);

        //Atualiza parametros
        r->currentSolution[i][0] = 1;

        while(true)
        {
            //Se nao tiver nenhum no ainda alocado na rota
            if(hLength(i, r->currentSolution) == 1)
            {
                //Sorteia o no
                randomNode = generateInitialNode(randomReal(0,1), r->cumProb);

                //Tira da capacidade do veiculo a demanda do cliente
                currentCapacity -= r->demand[randomNode-1];

                if(currentCapacity >= 0)
                {
                    //Pluga o no sorteado na proxima posicao da rota
                    r->currentSolution[i][j] = randomNode;
                    //Muda a alocacao binaria de 0 para 1 utilizada pelo no inicial
                    r->binaryAllocation[randomNode-1] = 1;
                    //Adianta o contador
                    j++;
                }
                else
                {
                    //Pluga o super source no final da rota
                    r->currentSolution[i][hLength(i, r->currentSolution)] = 1;
                    //Sai do laco
                    break;
                }
            }
            //Se so tiver o no inicial alocado na rota
            if(hLength(i, r->currentSolution) == 2)
            {
                //Pega o no mais proximo do anteriormente sorteado
                nextNode = getNearestNode(randomNode, r->distanceMatrix, r->binaryAllocation);

                //Tira da capacidade do veiculo a demanda do cliente
                currentCapacity -= r->demand[nextNode-1];

                if(currentCapacity >= 0)
                {
                    //Pluga o no sorteado na proxima posicao da rota
                    r->currentSolution[i][j] = nextNode;
                    //Muda a alocacao binaria de 0 para 1 utilizada pelo no inicial
                    r->binaryAllocation[nextNode-1] = 1;
                    //Adianta o contador
                    j++;
                }
                else
                {
                    //Pluga o super source no final da rota
                    r->currentSolution[i][hLength(i, r->currentSolution)] = 1;
                    //Sai do laco
                    break;
                }
            }

            if(hLength(i, r->currentSolution) >= 3)
            {
                //Pega o no mais proximo do anteriormente sorteado
                dist[0] = getNearestDistance(r->currentSolution[i][1], r->distanceMatrix, r->binaryAllocation);
                dist[1] = getNearestDistance(r->currentSolution[i][hLength(i, r->currentSolution) - 1], r->distanceMatrix, r->binaryAllocation);

                if(dist[0] <= dist[1])
                {
                    nextNode = getNearestNode(r->currentSolution[i][1], r->distanceMatrix, r->binaryAllocation);
                }
                else
                {
                    nextNode = getNearestNode(r->currentSolution[i][hLength(i, r->currentSolution) - 1], r->distanceMatrix, r->binaryAllocation);
                }

                //Tira da capacidade do veiculo a demanda do cliente
                currentCapacity -= r->demand[nextNode-1];

                if(currentCapacity >= 0)
                {
                    if(dist[0] >= dist[1])
                    {
                        //Pluga o no sorteado na proxima posicao da rota
                        r->currentSolution[i][hLength(i, r->currentSolution)] = nextNode;
                        //Muda a alocacao binaria de 0 para 1 utilizada pelo no inicial
                        r->binaryAllocation[nextNode-1] = 1;
                    }
                    else
                    {
                        //Desloca o vetor solucao para a direita para pode inserir o novo no
                        for(k = hLength(i, r->currentSolution) - 1; k >= 1; k--)
                        {
                            r->currentSolution[i][k+1] = r->currentSolution[i][k];
                        }

                        //Pluga o no sorteado no inicio da rota
                        r->currentSolution[i][1] = nextNode;
                        //Muda a alocacao binaria de 0 para 1 utilizada pelo no inicial
                        r->binaryAllocation[nextNode-1] = 1;
                    }
                }
                else
                {
                    //Pluga o super source no final da rota
                    r->currentSolution[i][hLength(i, r->currentSolution)] = 1;
                    //Sai do laco
                    break;
                }
            }
        }

        //Passa pra proxima rota
        i++;
        //Primeira posicao na proxima rota
        j = 1;
        //Repoe a capacidade do veiculo
        currentCapacity = r->maxCapacity;
    }
}

bool checkFeasibility(int capacity, int demand[NODES], int solution[DIM][SPACE])
{
    int i, j;
    int testCapacity[vLength(solution)];

    for(i = 0; i <= vLength(solution) - 1; i++)
    {
        testCapacity[i] = capacity;

        for(j = 1; j <= hLength(i, solution) - 1; j++)
        {
            testCapacity[i] = testCapacity[i] - demand[solution[i][j]-1];
        }

         if(testCapacity[i] < 0)
        {
            return(false);
        }
    }

    return(true);
}

int calculateResult(int dist[NODES][NODES], int solution[DIM][SPACE])
{
    int i, j, sum;

    sum = 0;

    for(i = 0; i <= vLength(solution) - 1; i++)
    {
        for(j = 0; j <= hLength(i, solution) - 2; j++)
        {
            sum += dist[solution[i][j]-1][solution[i][j+1]-1];
        }
    }

    return(sum);
}

void hSwap(int depth, bool echo, route *r)
{
    int i, route, node[2], nodepos[2], cursol, newsol;

    for(i = 0; i <= depth; i++)
    {
        //Calcula a solucao atual
        cursol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Seleciona uma rota que tenha pelo menos 2 clientes
        do
        {
            route = randomInteger(0, vLength(r->currentSolution) - 1);
        }
        while(hLength(route, r->currentSolution) <= 3);

        node[0] = r->currentSolution[route][randomInteger(1, hLength(route, r->currentSolution) - 2)];

        //Sorteia o segundo cliente de modo que nao seja igual ao primeiro
        do
        {
            node[1] = r->currentSolution[route][randomInteger(1, hLength(route, r->currentSolution) - 2)];
        }
        while(node[1] == node[0]);

        //Captura a posicao dos nos
        nodepos[0] = getNodeIndex(node[0], route, r->currentSolution);
        nodepos[1] = getNodeIndex(node[1], route, r->currentSolution);

        //Realiza o swap
        r->currentSolution[route][nodepos[0]] = node[1];
        r->currentSolution[route][nodepos[1]] = node[0];

        //Calcula a nova solucao do problema
        newsol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Caso a nova solucao seja melhor atualiza a solucao corrente e imprime a nova solucao
        if(newsol < cursol)
        {
            cursol = newsol;
            r->neighborhoodControl = 1;

            if(echo)
            {
                cout << "Updating best solution (hSwap): " << newsol << "\n";
                displaySolution(r->currentSolution);
            }
        }
        else
        {
            //Destroca os nos substituidos
            r->currentSolution[route][nodepos[0]] = node[0];
            r->currentSolution[route][nodepos[1]] = node[1];
        }
    }
}

void hLeftShift(int depth, bool echo, route *r)
{
    int i, j, route, first, cursol, newsol, node[SPACE];

    for(i = 0; i <= depth; i++)
    {
        //Calcula a solucao atual
        cursol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Seleciona uma rota que tenha pelo menos 3 clientes
        do
        {
            route = randomInteger(0, vLength(r->currentSolution) - 1);
        }
        while(hLength(route, r->currentSolution) <= 4);

        for(j = 1; j <= hLength(route, r->currentSolution) - 2; j++)
        {
            node[j-1] = r->currentSolution[route][j];
        }

        //Realiza o left shift
        first = r->currentSolution[route][1];

        for(j = 1; j <= hLength(route, r->currentSolution) - 2; j++)
        {
            r->currentSolution[route][j] = r->currentSolution[route][j+1];
        }

        r->currentSolution[route][hLength(route, r->currentSolution)-2] = first;

        //Calcula a nova solucao do problema
        newsol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Caso a nova solucao seja melhor atualiza a solucao corrente e imprime a nova solucao
        if(newsol < cursol)
        {
            cursol = newsol;
            r->neighborhoodControl = 1;

            if(echo)
            {
                cout << "Updating best solution (hLeftShift): " << newsol << "\n";
                displaySolution(r->currentSolution);
            }
        }
        else
        {
            //Destroca os nos substituidos
            for(j = 1; j <= hLength(route, r->currentSolution) - 2; j++)
            {
                r->currentSolution[route][j] = node[j-1];
            }
        }
    }
}

void hRightShift(int depth, bool echo, route *r)
{
    int i, j, route, last, cursol, newsol, node[SPACE];

    for(i = 0; i <= depth; i++)
    {
        //Calcula a solucao atual
        cursol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Seleciona uma rota que tenha pelo menos 3 clientes
        do
        {
            route = randomInteger(0, vLength(r->currentSolution) - 1);
        }
        while(hLength(route, r->currentSolution) <= 4);

        for(j = 1; j <= hLength(route, r->currentSolution) - 2; j++)
        {
            node[j-1] = r->currentSolution[route][j];
        }

        //Realiza o right shift
        last = r->currentSolution[route][hLength(route, r->currentSolution)-2];

        for(j = hLength(route, r->currentSolution) - 2; j >= 2; j--)
        {
            r->currentSolution[route][j] = r->currentSolution[route][j-1];
        }

        r->currentSolution[route][1] = last;

        //Calcula a nova solucao do problema
        newsol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Caso a nova solucao seja melhor atualiza a solucao corrente e imprime a nova solucao
        if(newsol < cursol)
        {
            cursol = newsol;
            r->neighborhoodControl = 1;

            if(echo)
            {
                cout << "Updating best solution (hRightShift): " << newsol << "\n";
                displaySolution(r->currentSolution);
            }
        }
        else
        {
            //Destroca os nos substituidos
            for(j = 1; j <= hLength(route, r->currentSolution) - 2; j++)
            {
                r->currentSolution[route][j] = node[j-1];
            }
        }
    }
}

void hInsert(int depth, bool echo, route *r)
{
    int i, j, route, node, nodepos, newpos, cursol, newsol;

    for(i = 0; i <= depth; i++)
    {
        //Calcula a solucao atual
        cursol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Seleciona uma rota que tenha pelo menos 3 clientes
        do
        {
            route = randomInteger(0, vLength(r->currentSolution) - 1);
        }
        while(hLength(route, r->currentSolution) <= 4);

        //Seleciona o cliente
        node = r->currentSolution[route][randomInteger(1, hLength(route, r->currentSolution) - 2)];

        //Captura a posicao do no
        nodepos = getNodeIndex(node, route, r->currentSolution);

        do
        {
            newpos = getNodeIndex(r->currentSolution[route][randomInteger(1, hLength(route, r->currentSolution) - 2)], route, r->currentSolution);
        }
        while(newpos == nodepos);

        if(newpos > nodepos)
        {
            for(j = nodepos; j < newpos; j++)
            {
                r->currentSolution[route][j] = r->currentSolution[route][j+1];
            }
        }
        else
        {
            for(j = nodepos; j > newpos; j--)
            {
                r->currentSolution[route][j] = r->currentSolution[route][j-1];
            }
        }

        r->currentSolution[route][newpos] = node;

        //Calcula a nova solucao do problema
        newsol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Caso a nova solucao seja melhor atualiza a solucao corrente e imprime a nova solucao
        if(newsol < cursol)
        {
            cursol = newsol;
            r->neighborhoodControl = 1;

            if(echo)
            {
                cout << "Updating best solution (hInsert): " << newsol << "\n";
                displaySolution(r->currentSolution);
            }
        }
        else
        {
            //Destroca os nos substituidos
            if(newpos > nodepos)
            {
                for(j = newpos; j > nodepos; j--)
                {
                    r->currentSolution[route][j] = r->currentSolution[route][j-1];
                }
            }
            else
            {
                for(j = newpos; j < nodepos; j++)
                {
                    r->currentSolution[route][j] = r->currentSolution[route][j+1];
                }
            }

            r->currentSolution[route][nodepos] = node;
        }
    }
}

void hInterchange(int depth, bool echo, route *r)
{
    int i, route[2], node[2], nodepos[2], cursol, newsol;

    for(i = 0; i <= depth; i++)
    {
        //Calcula a solucao atual
        cursol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Seleciona a primeira rota
        route[0] = randomInteger(0, vLength(r->currentSolution) - 1);

        //Sorteia a segunda rota
        do
        {
            route[1] = randomInteger(0, vLength(r->currentSolution) - 1);
        }
        while(route[1] == route[0]);

        //Sorteia os nos
        node[0] = r->currentSolution[route[0]][randomInteger(1, hLength(route[0], r->currentSolution) - 2)];
        node[1] = r->currentSolution[route[1]][randomInteger(1, hLength(route[1], r->currentSolution) - 2)];

        //Captura a posicao dos nos
        nodepos[0] = getNodeIndex(node[0], route[0], r->currentSolution);
        nodepos[1] = getNodeIndex(node[1], route[1], r->currentSolution);

        //Realiza o intercambio
        r->currentSolution[route[0]][nodepos[0]] = node[1];
        r->currentSolution[route[1]][nodepos[1]] = node[0];

        //Calcula a nova solucao do problema
        newsol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Caso a nova solucao seja melhor atualiza a solucao corrente e imprime a nova solucao
        if(newsol < cursol && checkFeasibility(r->maxCapacity, r->demand, r->currentSolution))
        {
            cursol = newsol;
            r->neighborhoodControl = 1;

            if(echo)
            {
                cout << "Updating best solution (hInterchange): " << newsol << "\n";
                displaySolution(r->currentSolution);
            }
        }
        else
        {
            //Destroca os nos substituidos
            r->currentSolution[route[0]][nodepos[0]] = node[0];
            r->currentSolution[route[1]][nodepos[1]] = node[1];
        }
    }
}

void hReallocation(int depth, bool echo, route *r)
{
    int i, j, k, route[2], node, froute, prevsol[2][SPACE], prevsize[2], nodepos[2], cursol, newsol;

    for(i = 0; i <= depth; i++)
    {
        //Calcula a solucao atual
        cursol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Sorteia a primeira rota > 3
        do
        {
            route[0] = randomInteger(0, vLength(r->currentSolution) - 1);
        }
        while(hLength(route[0], r->currentSolution) <= 3);

        //Sorteia a segunda rota
        do
        {
            route[1] = randomInteger(0, vLength(r->currentSolution) - 1);
        }
        while(route[1] == route[0]);

        //Seleciona os clientes
        node = r->currentSolution[route[0]][randomInteger(1, hLength(route[0], r->currentSolution) - 2)];

        //Captura a posicao do no de saida e de entrada
        nodepos[0] = getNodeIndex(node, route[0], r->currentSolution);
        nodepos[1] = randomInteger(1, hLength(route[1], r->currentSolution) - 1);

        // Tamanho da primeira rota
        froute = hLength(route[0], r->currentSolution);

        //Caso a rota tenha apenas um cliente, elimina a rota
        if(froute == 3)
        {
            for(j = 0; j <= hLength(route[0], r->currentSolution) - 1; j++)
            {
                r->currentSolution[route[0]][j] = 0;
            }

            prevsize[1] = hLength(route[1], r->currentSolution);

            //Armazenando as rotas
            for(j = 0; j <= prevsize[1] - 1; j++)
            {
                prevsol[1][j] = r->currentSolution[route[1]][j];
            }
        }
        else
        {
             prevsize[0] = hLength(route[0], r->currentSolution);
             prevsize[1] = hLength(route[1], r->currentSolution);

            for(j = 0; j <= 1; j++)
            {
                for(k = 0; k <= prevsize[j] - 1; k++)
                {
                    prevsol[j][k] = r->currentSolution[route[j]][k];
                }
            }
        }

        //Realiza a realocacao
        for(j = hLength(route[1], r->currentSolution); j >= nodepos[1]; j--)
        {
            r->currentSolution[route[1]][j+1] = r->currentSolution[route[1]][j];
        }

        for(j = nodepos[0]; j <= hLength(route[0], r->currentSolution); j++)
        {
            r->currentSolution[route[0]][j] = r->currentSolution[route[0]][j+1];
        }

        r->currentSolution[route[1]][nodepos[1]] = node;

        //Calcula a nova solucao do problema
        newsol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Caso a nova solucao seja melhor atualiza a solucao corrente e imprime a nova solucao
        if(newsol < cursol && checkFeasibility(r->maxCapacity, r->demand, r->currentSolution))
        {
            cursol = newsol;
            r->neighborhoodControl = 1;

            if(echo)
            {
                cout << "Updating best solution (hReallocation): " << newsol << "\n";
                displaySolution(r->currentSolution);
            }
        }
        else
        {
            if(froute == 3)
            {
                r->currentSolution[vLength(r->currentSolution)][0] = 1;
                r->currentSolution[vLength(r->currentSolution)][1] = nodepos[0];
                r->currentSolution[vLength(r->currentSolution)][2] = 1;

                for(j = 0; j <= prevsize[1] + 1; j++)
                {
                    r->currentSolution[route[1]][j] = 0;
                }

                for(j = 0; j <= prevsize[1] - 1; j++)
                {
                    r->currentSolution[route[1]][j] = prevsol[1][j];
                }
            }
            else
            {
                for(j = 0; j <= 1; j++)
                {
                    for(k = 0; k <= prevsize[j] + 1; k++)
                    {
                        r->currentSolution[route[j]][k] = 0;
                    }
                }

                for(j = 0; j <= 1; j++)
                {
                    for(k = 0; k <= prevsize[j] - 1; k++)
                    {
                        r->currentSolution[route[j]][k] = prevsol[j][k];
                    }
                }
            }
        }
    }
}

void hTraditionalCross(int depth, bool echo, route *r)
{
    int i, j, k, prinroute, node, route[2], aux[2][SPACE], cutsize, cursol, newsol;

    for(i = 0; i <= depth; i++)
    {
        //Calcula a solucao atual
        cursol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Seleciona a primeira rota
        do
        {
            route[0] = randomInteger(0, vLength(r->currentSolution) - 1);
        }
        while(hLength(route[0], r->currentSolution) <= 3);

        //Sorteia a segunda rota
        do
        {
            route[1] = randomInteger(0, vLength(r->currentSolution) - 1);
        }
        while(route[1] == route[0]);

        //Verifica a rota base para determinar a posicao de corte dos segmentos (no caso tem de ser compativel com as 2 rotas)
        if(hLength(route[0], r->currentSolution) >= hLength(route[1], r->currentSolution))
        {
            prinroute = route[1];
        }
        else
        {
            prinroute = route[0];
        }

        //Sorteia um no da rota base
        node = r->currentSolution[prinroute][randomInteger(1, hLength(prinroute, r->currentSolution) - 2)];

        //Determina o ponto de corte na rota base
        cutsize = hLength(prinroute, r->currentSolution) - getNodeIndex(node, prinroute, r->currentSolution);

        k = 0;

        for(j = 0; j <= 1; j++)
        {
            for(k = 0; k <= SPACE - 1; k++)
            {
                aux[j][k] = 0;
            }
        }

        k = 0;

        //Realiza o cruzamento tradicional
        for(j = hLength(route[0], r->currentSolution) - cutsize; j <= hLength(route[0], r->currentSolution); j++)
        {
            aux[0][k] = r->currentSolution[route[0]][j];
            k++;
        }

        k = 0;

        //Realiza o cruzamento tradicional
        for(j = hLength(route[1], r->currentSolution) - cutsize; j <= hLength(route[1], r->currentSolution); j++)
        {
            aux[1][k] = r->currentSolution[route[1]][j];
            k++;
        }

        k = hLength(route[0], r->currentSolution) - cutsize;

        for(j = 0; j <= hLength(1, aux) - 1; j++)
        {
            r->currentSolution[route[0]][k] = aux[1][j];
            k++;
        }

        k = hLength(route[1], r->currentSolution) - cutsize;

        for(j = 0; j <= hLength(0, aux) - 1; j++)
        {
            r->currentSolution[route[1]][k] = aux[0][j];
            k++;
        }

        //Calcula a nova solucao do problema
        newsol = calculateResult(r->distanceMatrix, r->currentSolution);

        //Caso a nova solucao seja melhor atualiza a solucao corrente e imprime a nova solucao
        if(newsol < cursol && checkFeasibility(r->maxCapacity, r->demand, r->currentSolution))
        {
            cursol = newsol;
            r->neighborhoodControl = 1;

            if(echo)
            {
                cout << "Updating best solution (hTraditionalCross): " << newsol << "\n";
                displaySolution(r->currentSolution);
            }
        }
        else
        {
            k = hLength(route[0], r->currentSolution) - cutsize;

            for(j = 0; j <= hLength(0, aux) - 1; j++)
            {
                r->currentSolution[route[0]][k] = aux[0][j];
                k++;
            }

            k = hLength(route[1], r->currentSolution) - cutsize;

            for(j = 0; j <= hLength(1, aux) - 1; j++)
            {
                r->currentSolution[route[1]][k] = aux[1][j];
                k++;
            }
        }
    }
}

void displaySolution(int solution[DIM][SPACE])
{
    int i, j;

    for(i = 0; i <= vLength(solution) - 1; i++)
    {
        for(j = 0; j <= hLength(i, solution) - 1; j++)
        {
            cout << solution[i][j] << " ";
        }

        cout << "\n";
    }
}

int getNodeIndex(int node, int route, int solution[DIM][SPACE])
{
    int i, j;

    for(i = 0; i <= vLength(solution) - 1; i++)
    {
        for(j = 0; j <= hLength(route, solution) - 1; j++)
        {
            if(node == solution[i][j])
            {
                return(j);
            }
        }
    }

    return(0);
}

void mainControl(int depth, bool echo, route *r)
{
    double sample, margin = 1.25;
    int iter = 0, maxIter = 1000000, newsol, bestsol = 1000000, dist = 0, maxDist = 10;
    int marginMod = 0;
    int depthMod = 0;
    int depthControl = depth;

    while(iter <= maxIter)
    {
        cout << "Current solution: " << calculateResult(r->distanceMatrix, r->currentSolution) << "\n";

        mainNeighborhood(depthControl, echo, r);

        newsol = calculateResult(r->distanceMatrix, r->currentSolution);

        if(newsol < bestsol)
        {
            bestsol = newsol;
            updateSolution(uBestEqualsCurrent, r);
            cout << "Updating best solution: " << calculateResult(r->distanceMatrix, r->bestSolution) << "\n";
            margin = 1.25;
            marginMod = 0;
            depthMod = 0;
            depthControl = depth;

        }
        else
        {
            dist++;
        }

        if(dist >= maxDist)
        {
            sample = randomReal(0,1);

            if(sample >= 0.5)
            {
                updateSolution(uDisturbanceEqualsCurrent, r);
            }
            else
            {
                updateSolution(uDisturbanceEqualsBest, r);
            }

            cout << "Disturbance solution: " << calculateResult(r->distanceMatrix, r->disturbanceSolution) << "\n";
            mainDisturbance(margin*calculateResult(r->distanceMatrix, r->bestSolution), 10000, r);

            marginMod++;
            depthMod++;

            if(marginMod >= 5)
            {
                margin += 0.05;
                cout << "Updating tolerance: " << margin*100 - 100 << "%\n";
                marginMod = 0;
            }

            if(depthMod >= 12)
            {
                depthControl += 300;
                cout << "Increasing search depth: " << depthControl << "\n";
                depthMod = 0;
            }

            dist = 0;
        }

        iter++;
    }
}

void mainNeighborhood(int depth, bool echo, route *r)
{
    START:
    r->neighborhoodControl = 0;

    hTraditionalCross(depth, echo, r);

    if(r->neighborhoodControl == 1)
    {
        goto START;
    }

    hInterchange(depth, echo, r);

    if(r->neighborhoodControl == 1)
    {
        goto START;
    }

    hReallocation(depth, echo, r);

    if(r->neighborhoodControl == 1)
    {
        goto START;
    }

    hLeftShift(depth, echo, r);

    if(r->neighborhoodControl == 1)
    {
        goto START;
    }

    hRightShift(depth, echo, r);

    if(r->neighborhoodControl == 1)
    {
        goto START;
    }

    hInsert(depth, echo, r);

    if(r->neighborhoodControl == 1)
    {
        goto START;
    }

    hSwap(depth, echo, r);

    if(r->neighborhoodControl == 1)
    {
        goto START;
    }
}

void mainDisturbance(double tol, int depth, route *r)
{
    int i, j, w = 0, newsol;
    int random[2];
    int dMax;
    double tolerance;

    tolerance = tol;

    for(i = 0; i <= depth; i++)
    {
        random[0] = randomInteger(1,7);
        random[1] = randomInteger(1,7);
        dMax = randomInteger(2,5);

        for(j = 0; j <= dMax; j++)
        {
            switch(random[j])
            {
                case 1:
                    dSwap(r);
                    break;
                case 2:
                    dLeftShift(r);
                    break;
                case 3:
                    dRightShift(r);
                    break;
                case 4:
                    dInsert(r);
                    break;
                case 5:
                    dInterchange(r);
                    break;
                case 6:
                    dReallocation(r);
                    break;
                case 7:
                    dTraditionalCross(r);
                    break;
            }
        }

        newsol = calculateResult(r->distanceMatrix, r->disturbanceSolution);

        if(newsol < tolerance)
        {
            tolerance = newsol;
            updateSolution(uCurrentEqualsDisturbance, r);
            w = 1;
        }

        if(!w)
        {
            updateSolution(uDisturbanceEqualsCurrent, r);
        }
    }
}

void dSwap(route *r)
{
    int route, node[2], nodepos[2];

    //Seleciona uma rota que tenha pelo menos 2 clientes
    do
    {
        route = randomInteger(0, vLength(r->disturbanceSolution) - 1);
    }
    while(hLength(route, r->disturbanceSolution) <= 3);

    //Sorteia a rota e o primeiro cliente
    node[0] =  r->disturbanceSolution[route][randomInteger(1, hLength(route, r->disturbanceSolution) - 2)];

    //Sorteia o segundo cliente de modo que nao seja igual ao primeiro
    do
    {
        node[1] = r->disturbanceSolution[route][randomInteger(1, hLength(route, r->disturbanceSolution) - 2)];
    }
    while(node[1] == node[0]);

    //Captura a posicao dos nos
    nodepos[0] = getNodeIndex(node[0], route, r->disturbanceSolution);
    nodepos[1] = getNodeIndex(node[1], route, r->disturbanceSolution);

    //Realiza o swap
    r->disturbanceSolution[route][nodepos[0]] = node[1];
    r->disturbanceSolution[route][nodepos[1]] = node[0];
}

void dLeftShift(route *r)
{
    int j, route, first;

    //Seleciona uma rota que tenha pelo menos 3 clientes
    do
    {
        route = randomInteger(0, vLength(r->disturbanceSolution) - 1);
    }
    while(hLength(route, r->disturbanceSolution) <= 4);

    //Realiza o left shift
    first = r->disturbanceSolution[route][1];

    for(j = 1; j <= hLength(route, r->disturbanceSolution) - 2; j++)
    {
        r->disturbanceSolution[route][j] = r->disturbanceSolution[route][j+1];
    }

    r->disturbanceSolution[route][hLength(route, r->disturbanceSolution)-2] = first;
}

void dRightShift(route *r)
{
    int j, route, last;

    //Seleciona uma rota que tenha pelo menos 3 clientes
    do
    {
        route = randomInteger(0, vLength(r->disturbanceSolution) - 1);
    }
    while(hLength(route, r->disturbanceSolution) <= 4);

    //Realiza o right shift
    last = r->disturbanceSolution[route][hLength(route, r->disturbanceSolution)-2];

    for(j = hLength(route, r->disturbanceSolution) - 2; j >= 2; j--)
    {
        r->disturbanceSolution[route][j] = r->disturbanceSolution[route][j-1];
    }

    r->disturbanceSolution[route][1] = last;
}

void dInsert(route *r)
{
    int j, route, node, nodepos, newpos;

    //Seleciona uma rota que tenha pelo menos 3 clientes
    do
    {
        route = randomInteger(0, vLength(r->disturbanceSolution) - 1);
    }
    while(hLength(route, r->disturbanceSolution) <= 4);

    //Seleciona o cliente
    node = r->disturbanceSolution[route][randomInteger(1, hLength(route, r->disturbanceSolution) - 2)];

    //Captura a posicao do no
    nodepos = getNodeIndex(node, route, r->disturbanceSolution);

    do
    {
        newpos = getNodeIndex(r->disturbanceSolution[route][randomInteger(1, hLength(route, r->disturbanceSolution) - 2)], route, r->disturbanceSolution);
    }
    while(newpos == nodepos);

    if(newpos > nodepos)
    {
        for(j = nodepos; j < newpos; j++)
        {
            r->disturbanceSolution[route][j] = r->disturbanceSolution[route][j+1];
        }
    }
    else
    {
        for(j = nodepos; j > newpos; j--)
        {
            r->disturbanceSolution[route][j] = r->disturbanceSolution[route][j-1];
        }
    }

    r->disturbanceSolution[route][newpos] = node;
}

void dInterchange(route *r)
{
    int route[2], node[2], nodepos[2];

    //Seleciona a primeira rota
    route[0] = randomInteger(0, vLength(r->disturbanceSolution) - 1);

    //Sorteia a segunda rota
    do
    {
        route[1] = randomInteger(0, vLength(r->disturbanceSolution) - 1);
    }
    while(route[1] == route[0]);

    //Sorteia os nos
    node[0] = r->disturbanceSolution[route[0]][randomInteger(1, hLength(route[0], r->disturbanceSolution) - 2)];
    node[1] = r->disturbanceSolution[route[1]][randomInteger(1, hLength(route[1], r->disturbanceSolution) - 2)];

    //Captura a posicao dos nos
    nodepos[0] = getNodeIndex(node[0], route[0], r->disturbanceSolution);
    nodepos[1] = getNodeIndex(node[1], route[1], r->disturbanceSolution);

    //Realiza o intercambio
    r->disturbanceSolution[route[0]][nodepos[0]] = node[1];
    r->disturbanceSolution[route[1]][nodepos[1]] = node[0];

    //Caso a nova solucao nao seja viavel retorna a antiga
    if(!checkFeasibility(r->maxCapacity, r->demand, r->disturbanceSolution))
    {
        //Destroca os nos substituidos
        r->disturbanceSolution[route[0]][nodepos[0]] = node[0];
        r->disturbanceSolution[route[1]][nodepos[1]] = node[1];
    }
}

void dReallocation(route *r)
{
    int j, k, route[2], node, froute, prevsol[2][SPACE], prevsize[2], nodepos[2];

    //Sorteia a primeira rota > 3
    do
    {
        route[0] = randomInteger(0, vLength(r->disturbanceSolution) - 1);
    }
    while(hLength(route[0], r->disturbanceSolution) <= 3);

    //Sorteia a segunda rota
    do
    {
        route[1] = randomInteger(0, vLength(r->disturbanceSolution) - 1);
    }
    while(route[1] == route[0]);

    //Seleciona os clientes
    node = r->disturbanceSolution[route[0]][randomInteger(1, hLength(route[0], r->disturbanceSolution) - 2)];

    //Captura a posicao do no de saida e de entrada
    nodepos[0] = getNodeIndex(node, route[0], r->disturbanceSolution);
    nodepos[1] = randomInteger(1, hLength(route[1], r->disturbanceSolution) - 1);

    // Tamanho da primeira rota
    froute = hLength(route[0], r->disturbanceSolution);

    //Caso a rota tenha apenas um cliente, elimina a rota
    if(froute == 3)
    {
        for(j = 0; j <= hLength(route[0], r->disturbanceSolution) - 1; j++)
        {
            r->disturbanceSolution[route[0]][j] = 0;
        }

        prevsize[1] = hLength(route[1], r->disturbanceSolution);

        //Armazenando as rotas
        for(j = 0; j <= prevsize[1] - 1; j++)
        {
            prevsol[1][j] = r->disturbanceSolution[route[1]][j];
        }
    }
    else
    {
        prevsize[0] = hLength(route[0], r->disturbanceSolution);
        prevsize[1] = hLength(route[1], r->disturbanceSolution);

        for(j = 0; j <= 1; j++)
        {
            for(k = 0; k <= prevsize[j] - 1; k++)
            {
                prevsol[j][k] = r->disturbanceSolution[route[j]][k];
            }
        }
    }

    //Realiza a realocacao
    for(j = hLength(route[1], r->disturbanceSolution); j >= nodepos[1]; j--)
    {
        r->disturbanceSolution[route[1]][j+1] = r->disturbanceSolution[route[1]][j];
    }

    for(j = nodepos[0]; j <= hLength(route[0], r->disturbanceSolution); j++)
    {
        r->disturbanceSolution[route[0]][j] = r->disturbanceSolution[route[0]][j+1];
    }

    r->disturbanceSolution[route[1]][nodepos[1]] = node;

    //Caso a nova solucao seja melhor atualiza a solucao corrente e imprime a nova solucao
    if(!checkFeasibility(r->maxCapacity, r->demand, r->disturbanceSolution))
    {
        if(froute == 3)
        {
            r->disturbanceSolution[vLength(r->disturbanceSolution)][0] = 1;
            r->disturbanceSolution[vLength(r->disturbanceSolution)][1] = nodepos[0];
            r->disturbanceSolution[vLength(r->disturbanceSolution)][2] = 1;

            for(j = 0; j <= prevsize[1] + 1; j++)
            {
                r->disturbanceSolution[route[1]][j] = 0;
            }

            for(j = 0; j <= prevsize[1] - 1; j++)
            {
                r->disturbanceSolution[route[1]][j] = prevsol[1][j];
            }
        }
        else
        {
            for(j = 0; j <= 1; j++)
            {
                for(k = 0; k <= prevsize[j] + 1; k++)
                {
                    r->disturbanceSolution[route[j]][k] = 0;
                }
            }

            for(j = 0; j <= 1; j++)
            {
                for(k = 0; k <= prevsize[j] - 1; k++)
                {
                    r->disturbanceSolution[route[j]][k] = prevsol[j][k];
                }
            }
        }
    }
}

void dTraditionalCross(route *r)
{
    int j, k, prinroute, node, route[2], aux[2][SPACE], cutsize;

    //Seleciona a primeira rota
    do
    {
        route[0] = randomInteger(0, vLength(r->disturbanceSolution) - 1);
    }
    while(hLength(route[0], r->disturbanceSolution) <= 3);

    //Sorteia a segunda rota
    do
    {
        route[1] = randomInteger(0, vLength(r->disturbanceSolution) - 1);
    }
    while(route[1] == route[0]);

    //Verifica a rota base para determinar a posicao de corte dos segmentos (no caso tem de ser compativel com as 2 rotas)
    if(hLength(route[0], r->disturbanceSolution) >= hLength(route[1], r->disturbanceSolution))
    {
        prinroute = route[1];
    }
    else
    {
        prinroute = route[0];
    }

    //Sorteia um no da rota base
    node = r->disturbanceSolution[prinroute][randomInteger(1, hLength(prinroute, r->disturbanceSolution) - 2)];

    //Determina o ponto de corte na rota base
    cutsize = hLength(prinroute, r->disturbanceSolution) - getNodeIndex(node, prinroute, r->disturbanceSolution);

    k = 0;

    for(j = 0; j <= 1; j++)
    {
        for(k = 0; k <= SPACE - 1; k++)
        {
            aux[j][k] = 0;
        }
    }

    k = 0;

    //Realiza o cruzamento tradicional
    for(j = hLength(route[0], r->disturbanceSolution) - cutsize; j <= hLength(route[0], r->disturbanceSolution); j++)
    {
        aux[0][k] = r->disturbanceSolution[route[0]][j];
        k++;
    }

    k = 0;

    //Realiza o cruzamento tradicional
    for(j = hLength(route[1], r->disturbanceSolution) - cutsize; j <= hLength(route[1], r->disturbanceSolution); j++)
    {
        aux[1][k] = r->disturbanceSolution[route[1]][j];
        k++;
    }

    k = hLength(route[0], r->disturbanceSolution) - cutsize;

    for(j = 0; j <= hLength(1, aux) - 1; j++)
    {
        r->disturbanceSolution[route[0]][k] = aux[1][j];
        k++;
    }

    k = hLength(route[1], r->disturbanceSolution) - cutsize;

    for(j = 0; j <= hLength(0, aux) - 1; j++)
    {
        r->disturbanceSolution[route[1]][k] = aux[0][j];
        k++;
    }

    //Caso a nova solucao seja melhor atualiza a solucao corrente e imprime a nova solucao
    if(!checkFeasibility(r->maxCapacity, r->demand, r->disturbanceSolution))
    {
        k = hLength(route[0], r->disturbanceSolution) - cutsize;

        for(j = 0; j <= hLength(0, aux) - 1; j++)
        {
            r->disturbanceSolution[route[0]][k] = aux[0][j];
            k++;
        }

        k = hLength(route[1], r->disturbanceSolution) - cutsize;

        for(j = 0; j <= hLength(1, aux) - 1; j++)
        {
            r->disturbanceSolution[route[1]][k] = aux[1][j];
            k++;
        }
    }
}

void updateSolution(int type, route *r)
{
    int i, j;

    if(type == uBestEqualsCurrent)
    {
        for(i = 0; i <= DIM - 1; i++)
        {
            for(j = 0; j <= SPACE - 1; j++)
            {
                r->bestSolution[i][j] = 0;
            }
        }

        for(i = 0; i <= vLength(r->currentSolution) - 1; i++)
        {
            for(j = 0; j <= hLength(i, r->currentSolution) - 1; j++)
            {
                r->bestSolution[i][j] = r->currentSolution[i][j];
            }
        }
    }

    if(type == uCurrentEqualsBest)
    {
        for(i = 0; i <= DIM - 1; i++)
        {
            for(j = 0; j <= SPACE - 1; j++)
            {
                r->currentSolution[i][j] = 0;
            }
        }

        for(i = 0; i <= vLength(r->bestSolution) - 1; i++)
        {
            for(j = 0; j <= hLength(i, r->bestSolution) - 1; j++)
            {
                r->currentSolution[i][j] = r->bestSolution[i][j];
            }
        }
    }

    if(type == uDisturbanceEqualsCurrent)
    {
        for(i = 0; i <= DIM - 1; i++)
        {
            for(j = 0; j <= SPACE - 1; j++)
            {
                r->disturbanceSolution[i][j] = 0;
            }
        }

        for(i = 0; i <= vLength(r->currentSolution) - 1; i++)
        {
            for(j = 0; j <= hLength(i, r->currentSolution) - 1; j++)
            {
                r->disturbanceSolution[i][j] = r->currentSolution[i][j];
            }
        }
    }

    if(type == uCurrentEqualsDisturbance)
    {
        for(i = 0; i <= DIM - 1; i++)
        {
            for(j = 0; j <= SPACE - 1; j++)
            {
                r->currentSolution[i][j] = 0;
            }
        }

        for(i = 0; i <= vLength(r->disturbanceSolution) - 1; i++)
        {
            for(j = 0; j <= hLength(i, r->disturbanceSolution) - 1; j++)
            {
                r->currentSolution[i][j] = r->disturbanceSolution[i][j];
            }
        }
    }

    if(type == uDisturbanceEqualsBest)
    {
        for(i = 0; i <= DIM - 1; i++)
        {
            for(j = 0; j <= SPACE - 1; j++)
            {
                r->disturbanceSolution[i][j] = 0;
            }
        }

        for(i = 0; i <= vLength(r->bestSolution) - 1; i++)
        {
            for(j = 0; j <= hLength(i, r->bestSolution) - 1; j++)
            {
                r->disturbanceSolution[i][j] = r->bestSolution[i][j];
            }
        }
    }
}
