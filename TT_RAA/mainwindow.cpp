#include "mainwindow.h"
#include "ui_mainwindow.h"


QCoreApplication *app = QCoreApplication::instance();

Kinematic k1,k2;
InverseKinematic Ik1, Ik2;

ax12a m(0);
ax12a m1(1);
ax12a m2(2);
ax12a m3(3);

int home[4] = {0, 0, 0, 0};
double poslineal[3] = {0,0,0};
double posangular[3] = {0,0,0};

int pxcont, pycont, pzcont;

int speed;

double rad = (M_PI/180);
double grad = 0.29297;
double q1, q2, q3, q4;
double q_sliders[4] = {0, 204, 512, 512};
int px, py, pz, q1inv, q2inv, q3inv, q1motor, q2motor, q3motor;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QPixmap ipn("/home/shiza/Escritorio/TT_RAA_Control/logo-ipn-guinda.png");
    QPixmap upiiz("/home/shiza/Escritorio/TT_RAA_Control/UPIIZ.png");

    ui->logo_ipn->setPixmap(ipn.scaled(80,120));
    ui->logo_upiiz->setPixmap(upiiz);

    objWorldGL = new WorldGL(ui->animacion);
    objWorldGL->resize(550,525); // el resize coincide con el tamaño del openGLWidget
    connect(ui->eslabon1,SIGNAL(valueChanged(int)),this,SLOT(setRobotConfig()));
    connect(ui->eslabon2,SIGNAL(valueChanged(int)),this,SLOT(setRobotConfig()));
    connect(ui->eslabon3,SIGNAL(valueChanged(int)),this,SLOT(setRobotConfig()));
    connect(ui->eslabon4,SIGNAL(valueChanged(int)),this,SLOT(setRobotConfig()));

    loadRobot();

    //app->processEvents();

    //Iniciando puerto
    m.initport();
    m1.initport();
    m2.initport();
    m3.initport();

    //Se encienden los pares
    m.enabletorque();
    m1.enabletorque();
    m2.enabletorque();
    m3.enabletorque();

    //Se restringe la velocidad de cada motor
    m.setspeed(speed = 165);
    m1.setspeed(speed = 165);
    m2.setspeed(speed = 165);
    m3.setspeed(speed = 165);

    //Se mueven los eslabones a la posicion inicial
    m.setposition(home[0]);
    m1.setposition(home[1]);
    m2.setposition(home[2]);
    m3.setposition(home[3]);

    //Inicia los sliders en la posición inicial
    ui->eslabon1->setValue(home[0]);
    ui->eslabon2->setValue(home[1]);
    ui->eslabon3->setValue(home[2]);
    ui->eslabon4->setValue(home[3]);

    //Se leen los valores de los motores y se guardan
    m.getposition();
    m1.getposition();
    m2.getposition();

    k1.DirecKinect(q1,q2,q3,k1.px,k1.py,k1.pz);

    ui->q1->display(q1);
    ui->pos_x->display(k1.px);
    poslineal[0] = k1.px;

    ui->q2->display(q2-90);
    ui->pos_y->display(k1.py);
    poslineal[1] = k1.py;

    ui->q3->display(q3);
    ui->pos_z->display(k1.pz);
    poslineal[2] = k1.pz;

    pxcont = poslineal[0];
    pycont = poslineal[1];
    pzcont = poslineal[2];

}

MainWindow::~MainWindow()
{
    delete ui;

    m.turnoff();
    m1.turnoff();
    m2.turnoff();
    m3.turnoff();
}

//Funciones para la animacion

void MainWindow::setRobotConfig()
{
    if (!objWorldGL->robotVector.empty()){
        float value;
        vector <float> config;
        value = ui->eslabon1->value()+adjust1;
        config.push_back(value);
        if (ui->eslabon1->isEnabled()){
            value = ui->eslabon2->value()+adjust2;
            config.push_back(value);
        }
        if (ui->eslabon3->isEnabled()){
            value = ui->eslabon3->value()+adjust3;
            config.push_back(value);
        }
        if (ui->eslabon4->isEnabled()){
            value = ui->eslabon4->value()+adjust4;
            config.push_back(value);
        }
        objWorldGL->setRobotConfig(config,0);
        objWorldGL->updateGL();

    }
//    return config;
}

void MainWindow::loadRobot(){
    QString fileName("/home/shiza/Escritorio/Animacion/Render_robot/momosbot_chidisimo_sin_comas.rbt");

    QFile file(fileName);
    if (file.open(QIODevice::ReadOnly)){
        newRobot = new Robot;
            while (!file.atEnd()) {
                QString line = file.readLine();
                QTextStream lineIn(&line);
                QString aux;
                coordVector cVectorAux;
                triFace triFaceAux;
                float terminatorAux;
                if (line.contains("Body")) {
                    newBody = new Body;
                    lineIn >> aux >> newBody->bodyName >> aux;
                    while (!line.contains("}")){
                        line = file.readLine();
                        QTextStream lineIn(&line);
                        if (line.contains("Transform")){
                            lineIn >> aux >> newBody->transform[0]>>newBody->transform[1]>>newBody->transform[2];
                        }
                        if (line.contains("Rotation")){
                            lineIn >> aux >> newBody->rotation[0]>>newBody->rotation[1]>>newBody->rotation[2]>>newBody->rotation[3];
                        }
                        if (line.contains("Center")){
                            lineIn >> aux >> newBody->center[0]>>newBody->center[1]>>newBody->center[2];
                        }
                        if (line.contains("points")){
                            line = file.readLine();
                            while (!line.contains("]")){
                                QTextStream lineIn(&line);
                                lineIn >> cVectorAux.x >> cVectorAux.y >> cVectorAux.z;
                                newBody->points.push_back(cVectorAux);
                                line = file.readLine();
                            }
                        }
                        if (line.contains("faceIndex")){
                            line = file.readLine();
                            while (!line.contains("]")){
                                QTextStream lineIn(&line);
                                lineIn >> triFaceAux.c1 >> triFaceAux.c2 >> triFaceAux.c3 >> terminatorAux;
                                // If there is a quad make two triangles instead
                                if (terminatorAux<0){
                                    newBody->indexTriFaces.push_back(triFaceAux);
                                }
                                else {
                                    newBody->indexTriFaces.push_back(triFaceAux);
                                    triFaceAux.c2 = triFaceAux.c3;
                                    triFaceAux.c3 = terminatorAux;
                                    newBody->indexTriFaces.push_back(triFaceAux);
                                }
                                line = file.readLine();
                            }
                        }
                    }
                    // Create a random color for each body
                    newBody->color.r = rand() % 155 + 100;
                    newBody->color.g = rand() % 155;
                    newBody->color.b = rand() % 50;
                    newBody->adjustPoints();
                    // Calculate face normals
                    newBody->calculateNormals();
                    newBody->createFaces();
//                    newBody->buildPQPmodel();
                    newRobot->appendBody(*newBody);
                }
                if (line.contains("Joint")) {
                    newJoint = new Joint;
                    lineIn >> aux >> newJoint->jointName >> aux;
                    while (!line.contains("}")) {
                        line = file.readLine();
                        QTextStream lineIn(&line);
                        if (line.contains("jointType")){
                            lineIn >> aux >> newJoint->type;

                        }
                        if (line.contains("jointFather")){
                            lineIn >> aux >> newJoint->fatherString;
                        }
                        if (line.contains("jointChild")){
                            lineIn >> aux;
                            while (!lineIn.atEnd()){
                            lineIn >> aux;
                            newJoint->childString.push_back(aux);
                            }
                        }
                        if (line.contains("directorAxis")){
                            lineIn >> aux >> newJoint->jointAxis.x >> newJoint->jointAxis.y >> newJoint->jointAxis.z;
                        }
                        if (line.contains("limits")){
                            lineIn >> aux >> newJoint->limits.min >> newJoint->limits.max;
                        }
                        if (line.contains("value")){
                            lineIn >> aux >> newJoint->value;
                        }
                    }
                    newRobot->appendJoint(*newJoint);
                }
            }
    }
    file.close();
    newRobot->getJointLimits();
    objWorldGL->appendRobot(*newRobot);
    vector <int> lMax = newRobot->jointLimitsMax;
    vector <int> lMin = newRobot->jointLimitsMin;
    //setSlidersLimits(lMax,lMin);
}


//Programación de los sliders del modo de operación por eslabón

void MainWindow::on_eslabon1_sliderMoved(int position)
{
    q_sliders[0] = (position*grad-150)*rad;
    //Se manda la posicion al motor para que se mueva
    m.setposition(position);

    //Cada que se mueva el slider se leen las posiciones de los motores
    m.getposition();
    m1.getposition();
    m2.getposition();
    m3.getposition();

    //Se asignan las posiciones a variables para la cinematica directa
    q1 = (m.dxl_present_position*grad-150)*rad;
    q2 = (m1.dxl_present_position*grad-150+90)*rad;
    q3 = (m2.dxl_present_position*grad-150)*rad;
    q4 = (m3.dxl_present_position*grad-150)*rad;

    //Se introducen estos valores al metodo de la cinematica directa
    k1.DirecKinect(q1,q2,q3,k1.px,k1.py,k1.pz);

    //Se guardan en global los valores de angulo y posicion
    posangular[0] = q1;
    posangular[1] = q2;
    posangular[2] = q3;

    poslineal[0] = k1.px;
    poslineal[1] = k1.py;
    poslineal[2] = k1.pz;

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x->display(k1.px);
    ui->pos_y->display(k1.py);
    ui->pos_z->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q1->display((position*grad)-(home[0]*grad));
}


void MainWindow::on_eslabon2_sliderMoved(int position)
{
    q_sliders[1] = (position*grad-150+90)*rad;

    //Se revisa que el robot no vaya a entrar en singularidad
    k1.hitbox(k1.px,k1.py,k1.pz,k1.hit);
    k2.DirecKinect(q_sliders[0],q_sliders[1],q_sliders[2],k2.px,k2.py,k2.pz);
    k2.hitbox(k2.px,k2.py,k2.pz,k2.hit);

    //Se manda la posicion al motor para que se mueva
    if(!(k1.hit && k2.hit)){
        m1.setposition(position);
    }

    //Cada que se mueva el slider se leen las posiciones de los motores
    m.getposition();
    m1.getposition();
    m2.getposition();
    m3.getposition();

    //Se asignan las posiciones a variables para la cinematica directa
    q1 = (m.dxl_present_position*grad-150)*rad;
    q2 = (m1.dxl_present_position*grad-150+90)*rad;
    q3 = (m2.dxl_present_position*grad-150)*rad;
    q4 = (m3.dxl_present_position*grad-150)*rad;

    //Se introducen estos valores al metodo de la cinematica directa
    k1.DirecKinect(q1,q2,q3,k1.px,k1.py,k1.pz);

    //Se guardan en global los valores de angulo y posicion
    posangular[0] = q1;
    posangular[1] = q2;
    posangular[2] = q3;

    poslineal[0] = k1.px;
    poslineal[1] = k1.py;
    poslineal[2] = k1.pz;

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x->display(k1.px);
    ui->pos_y->display(k1.py);
    ui->pos_z->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q2->display((position*grad)-((home[1]+308)*grad));
}


void MainWindow::on_eslabon3_sliderMoved(int position)
{

    q_sliders[2] = (position*grad-150)*rad;

    //Se revisa que el robot no vaya a entrar en singularidad
    k1.hitbox(k1.px,k1.py,k1.pz,k1.hit);
    k2.DirecKinect(q_sliders[0],q_sliders[1],q_sliders[2],k2.px,k2.py,k2.pz);
    k2.hitbox(k2.px,k2.py,k2.pz,k2.hit);

    //Se manda la posicion al motor para que se mueva
    if(!(k1.hit && k2.hit)){
        m2.setposition(position);
    }

    //Cada que se mueva el slider se leen las posiciones de los motores
    m.getposition();
    m1.getposition();
    m2.getposition();
    m3.getposition();

    //Se asignan las posiciones a variables para la cinematica directa
    q1 = (m.dxl_present_position*grad-150)*rad;
    q2 = (m1.dxl_present_position*grad-150+90)*rad;
    q3 = (m2.dxl_present_position*grad-150)*rad;
    q4 = (m3.dxl_present_position*grad-150)*rad;

    //Se introducen estos valores al metodo de la cinematica directa
    k1.DirecKinect(q1,q2,q3,k1.px,k1.py,k1.pz);

    //Se guardan en global los valores de angulo y posicion
    posangular[0] = q1;
    posangular[1] = q2;
    posangular[2] = q3;

    poslineal[0] = k1.px;
    poslineal[1] = k1.py;
    poslineal[2] = k1.pz;

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x->display(k1.px);
    ui->pos_y->display(k1.py);
    ui->pos_z->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q3->display((position*grad)-(home[2]*grad));
}


void MainWindow::on_eslabon4_sliderMoved(int position)
{
    //Se manda la posicion al motor para que se mueva
    m3.setposition(position);
}



//Programación de los botones para el modo de operación por espacio de la herramienta

void MainWindow::on_xplus_clicked()
{
    //Se revisa que el robot no vaya a entrar en singularidad
    Ik1.hitbox(pxcont, pycont, pzcont, Ik1.hit);

    //Incrementa en uno el valor de contador
    pxcont = pxcont + 1;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se guardan en global los valores de angulo y posicion
    posangular[0] = Ik1.q1;
    posangular[1] = Ik1.q2;
    posangular[2] = Ik1.q3;

    poslineal[0] = pxcont;
    poslineal[1] = pxcont;
    poslineal[2] = pxcont;

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad)+150;
    q2inv = (Ik1.q2/rad)+150-90;
    q3inv = (Ik1.q3/rad)+150;

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Se muestra en HMI
    ui->pos_x->display(pxcont);

    ui->q1->display(q1inv - 150);
    ui->q2->display(q2inv - 150);
    ui->q3->display(q3inv - 150);

}


void MainWindow::on_xminus_clicked()
{
    //Incrementa en uno el valor de contador
    pxcont--;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se guardan en global los valores de angulo y posicion
    posangular[0] = Ik1.q1;
    posangular[1] = Ik1.q2;
    posangular[2] = Ik1.q3;

    poslineal[0] = pxcont;
    poslineal[1] = pxcont;
    poslineal[2] = pxcont;

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad)+150;
    q2inv = (Ik1.q2/rad)+150-90;
    q3inv = (Ik1.q3/rad)+150;

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Se muestra en HMI
    ui->pos_x->display(pxcont);

    ui->q1->display(q1inv - 150);
    ui->q2->display(q2inv - 150);
    ui->q3->display(q3inv - 150);
}


void MainWindow::on_yplus_clicked()
{
    //Incrementa en uno el valor de contador
    pycont++;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se guardan en global los valores de angulo y posicion
    posangular[0] = Ik1.q1;
    posangular[1] = Ik1.q2;
    posangular[2] = Ik1.q3;

    poslineal[0] = pxcont;
    poslineal[1] = pxcont;
    poslineal[2] = pxcont;

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad)+150;
    q2inv = (Ik1.q2/rad)+150-90;
    q3inv = (Ik1.q3/rad)+150;

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Se muestra en HMI
    ui->pos_y->display(pycont);

    ui->q1->display(q1inv - 150);
    ui->q2->display(q2inv - 150);
    ui->q3->display(q3inv - 150);
}


void MainWindow::on_yminus_clicked()
{
    //Incrementa en uno el valor de contador
    pycont--;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se guardan en global los valores de angulo y posicion
    posangular[0] = Ik1.q1;
    posangular[1] = Ik1.q2;
    posangular[2] = Ik1.q3;

    poslineal[0] = pxcont;
    poslineal[1] = pxcont;
    poslineal[2] = pxcont;

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad)+150;
    q2inv = (Ik1.q2/rad)+150-90;
    q3inv = (Ik1.q3/rad)+150;

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Se muestra en HMI
    ui->pos_y->display(pycont);

    ui->q1->display(q1inv - 150);
    ui->q2->display(q2inv - 150);
    ui->q3->display(q3inv - 150);
}


void MainWindow::on_zplus_clicked()
{
    //Incrementa en uno el valor de contador
    pzcont++;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se guardan en global los valores de angulo y posicion
    posangular[0] = Ik1.q1;
    posangular[1] = Ik1.q2;
    posangular[2] = Ik1.q3;

    poslineal[0] = pxcont;
    poslineal[1] = pxcont;
    poslineal[2] = pxcont;

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad)+150;
    q2inv = (Ik1.q2/rad)+150-90;
    q3inv = (Ik1.q3/rad)+150;

    q1motor = abs(q1inv/grad);
    q2motor = abs(q2inv/grad);
    q3motor = abs(q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Se muestra en HMI
    ui->pos_z->display(pzcont);

    ui->q1->display(q1inv - 150);
    ui->q2->display(q2inv - 150);
    ui->q3->display(q3inv - 150);
}


void MainWindow::on_zminus_clicked()
{
    //Incrementa en uno el valor de contador
    pzcont--;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se guardan en global los valores de angulo y posicion
    posangular[0] = Ik1.q1;
    posangular[1] = Ik1.q2;
    posangular[2] = Ik1.q3;

    poslineal[0] = pxcont;
    poslineal[1] = pxcont;
    poslineal[2] = pxcont;

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad)+150;
    q2inv = (Ik1.q2/rad)+150-90;
    q3inv = (Ik1.q3/rad)+150;

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Se muestra en HMI
    ui->pos_z->display(pzcont);

    ui->q1->display(q1inv - 150);
    ui->q2->display(q2inv - 150);
    ui->q3->display(q3inv - 150);
}


void MainWindow::on_home_clicked()
{
    //Se regresa a todos los motores a posición de home
    m.setposition(home[0]);
    m1.setposition(home[1]);
    m2.setposition(home[2]);
    m3.setposition(home[3]);

    //Se reinician los valores de los sliders
    ui->eslabon1->setValue(home[0]);
    ui->eslabon2->setValue(home[1]);
    ui->eslabon3->setValue(home[2]);
    ui->eslabon4->setValue(home[3]);
}


void MainWindow::on_pestanas_tabBarClicked(int index)
{
    int e1, e2, e3;

    if(index == 0){         //Eslabones

        //Se introducen valores globales al metodo de la cinematica directa
        k1.DirecKinect(posangular[0],posangular[1],posangular[2],k1.px,k1.py,k1.pz);

        poslineal[0] = k1.px;
        poslineal[1] = k1.py;
        poslineal[2] = k1.pz;

        //Se convierten los valores para reasignar la posicion de los sliders
        q1inv = (posangular[0]/rad)+150;
        q2inv = (posangular[1]/rad)+150-90;
        q3inv = (posangular[2]/rad)+150;

        e1 = (q1inv/grad);
        e2 = (q2inv/grad);
        e3 = (q3inv/grad);

        ui->eslabon1->setValue(e1);
        ui->eslabon2->setValue(e2);
        ui->eslabon3->setValue(e3);
    }
    else if(index == 1){    //Herramienta

        pxcont = poslineal[0];
        pycont = poslineal[1];
        pzcont = poslineal[2];

        //Entran a la inversa
        Ik1.InverseKinect(poslineal[0],poslineal[1],poslineal[2],Ik1.q1,Ik1.q2,Ik1.q3);

        posangular[0] = Ik1.q1;
        posangular[1] = Ik1.q2;
        posangular[2] = Ik1.q3;
    }
    else{                   //Trayectorias

    }
}

