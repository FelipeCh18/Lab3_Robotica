# Laboratorio 3 - "Robótica de Desarrollo, Intro a ROS"


***INTEGRANTES***

* Marco Antonio Quimbay Dueñas
* Felipe Chaves Delgadillo

## Metodología

El Laboratorio 3 se desarrolló a partir del [repositorio del profesor Pedro Cárdenas](https://github.com/PedroFCardenas/Intro_Ros) que amplía los contenidos incluidos en el [respositorio del Ingeniero Felipe Gonzales](https://github.com/fegonzalez7/rob_unal_clase2), ofreciendo una guía para facilitar la instalación y configuración del software ROS en los equipos del grupo. Se llevó a cabo un ejercicio inicial, similar al concepto de "hola mundo" en el programa, donde se clonó un repositorio específico en los directorios de trabajo. A través de comandos en la consola, se logró controlar una tortuga en una interfaz gráfica, estableciendo así las bases para comprender las comunicaciones entre nodos en ROS y prepararse para futuras interacciones entre programas en diferentes lenguajes.

Este ejercicio inicial no solo sirvió para familiarizarse con el funcionamiento básico de ROS, sino también como una introducción práctica a los conceptos fundamentales de comunicación entre nodos y control de robots. Además, sentó las bases para futuras actividades en las que se explorarían interacciones más complejas y la integración de programas en diferentes entornos de desarrollo dentro de ROS.

## MATLAB

Para la primera parte del laboratorio se pide cumplir con las tareas indicadas: escribir un script en Matlab que utilice la instrucción `rossubscriber` para suscribirse al tópico de pose de la simulación de turtle1, empleando la opción `LatestMessage` para capturar el último mensaje obtenido. Además, se necesita investigar los servicios de `turtlesim` disponibles para modificar la pose de la tortuga y enviar los valores asociados a la pose de turtle1. Por último, se debe indagar cómo finalizar el nodo maestro en Matlab para asegurar una terminación adecuada del programa.

PRIMER PROGRAMA
```
%%
rosinit; %Conexion al nodo maestro
%%
velPub = rospublisher('/turtle1/cmd_vel','geometry_msgs/Twist'); %Creacion publicador
velMsg = rosmessage(velPub); %Creacion de mensaje
%%
velMsg.Linear.X = 1; %Valor del mensaje
send(velPub,velMsg); %Envio
pause(1)
rosshutdown;
```

SUSCRIBE ROS
```
clc %Limpiar la ventana de comandos
%%
rosinit; % Establecer la conección con el máster de ROS para permitir la comunicación con matlab
%%
%Creación del publicador de mensajes para el envío de los comandos de velocidad de la tortuga
velPub = rospublisher('/turtle1/cmd_vel','geometry_msgs/Twist');
%Creación de una variable asociada con el publicador donde se guardarán los mensajes para el envío de comandos de velocidad de la tortuga. 
velMsg = rosmessage(velPub);
%Creación del suscriptor de mensajes, recibirá la posiión actual de la tortuga
velSub = rossubscriber("/turtle1/pose","turtlesim/Pose");
%%
%Configuración de la velocidad lineal en el mensaje de velocidad creado.
velMsg.Linear.X = 1;
%Envio del mensaje de velocidad a través del pubicador. (Hace que la tortuga se mueva)
send(velPub,velMsg);
%Pausa la ejecucion de código durante un segundo para permitir el movimiento de la tortuga antes de continuar
pause(1)
%Captura el último mensaje recibido a través del suscriptor creado, para poder conocer la nueva posición de la tortuga
Msg=velSub.LatestMessage;
%Cierra la conexión con el nodo maestro de ROS
rosshutdown;
```


### En caso de querer modificar la pose de la tortuga, es útil conocer los siguientes parámetros de los mensajes tipo twist que permiten hacer modificaciones en esta.
```

msg.Linear.X = [Float];  % Definición de posición X
msg.Linear.Y =  [Float]; % Definición de posición Y
msg.Linear.Z = [Float];  % Definición de posición Z
msg.Angular.X = [Float]; % Definición de orientación X
msg.Angular.Y = [Float]; % Definición de orientación Y
msg.Angular.Z = [Float]; % Definición de orientación Z 
```
## Python

En esta parte del laboratorio, se requirió desarrollar un script dentro del paquete hello_turtle para habilitar el control de una tortuga del paquete turtlesim utilizando el teclado (myTeleopKey).

Para poder cumplir con este requerimiento del laboratorio se realizó el siguiente código:

Para comenzar se importan las librerias necesarias para el trabajo con los nodos de ROS sugeridas por la guía así como las necesarias para el manejo de la tortuga a través del teclado:
```
import rospy #Para la interacción entre python y ROS
from geometry_msgs.msg import Twist #Para el uso de comandos de velocidad y servicios de teletransporte 
from turtlesim.srv import TeleportAbsolute, TeleportRelative #Para el uso de servicios de teletransporte relativo y absoluto
from std_srvs.srv import Empty #Para el uso del módulo
import termios,sys,os %Para configurar la intercción con la terminal
from numpy import pi %Para el uso del número pi
```

La siquiente sección define algunas constantes y variables que se utilizarán en el código, como los nombres de los tópicos de ROS relacionados con el movimiento y el teletransporte de la tortuga
```
TERMIOS = termios 

cmd_vel_topic = '/turtle1/cmd_vel'
teleport_ab = '/turtle1/teleport_absolute'
teleport_rel = '/turtle1/teleport_relative'
```

La función a continuaci´´on fue obtenida de un [blog de python](http://python4fun.blogspot.com/2008/06/get-key-press-in-python.html), ofrece la funcionalidad para obtener la entrada del teclado sin bloquear la ejecución del programa. Configurando temporalmente los atributos del terminal y luego restaurándolos después de obtener la entrada del teclado.
```
def getkey():
    fd = sys.stdin.fileno() #Obtiene el descriptor del archivo
    old = termios.tcgetattr(fd) #Guarda la configuración actual de la terminal 
    new = termios.tcgetattr(fd) #Copia la configuración actual de la terminal
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO #Modifica los atributos de la terminal para deshabilitar el modo canónico y el eco de entrada
    new[6][TERMIOS.VMIN] = 1 #Establece en 1 el número mínimo de caracteres leídos antes de que la función devuelva la entrada.
    new[6][TERMIOS.VTIME] = 0 #Establece en 0 el tiempo de espera, para que la lectura de las teclas sea prácticamente instantanea
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new) #Aplica los cambios realizados a la terminal
    c = None #Inicializa una variable para guardar la tecla leída
    try:
        c = os.read(fd, 1) #Lee la tecla y lo almacena en "c"
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old) #Restaura los atributos originales de la terminal
    return c
```

Luego, la sección que procede es responsable de interpretar la entrada del teclado y llamar a la función correspondiente para enviar los comandos de velocidad adecuados o realizar el teletransporte.
```
def tareaARealizar(): 
    tecla = getkey() #Obtiene la tecla presionada
    if tecla == b'w': 
        publicarVel(1,0) #publica un mensaje que hace hace que la tortuga avance en la dirección x
    elif tecla == b's':    
        publicarVel(-1,0) #publica un mensaje que hace hace que la tortuga retroceda en la dirección x
    elif tecla == b'd':
        publicarVel(0,-1) #publica un mensaje que hace que la tortuga gire CW respecto al eje z.
    elif tecla == b'a':
        publicarVel(0,1) #publica un mensaje que hace que la tortuga gire CCW respecto al eje z.
    elif tecla == b'r':
        teleport('home') #hace que la tortuga vuelva teletransportandose a la posición y orientación originales
    elif tecla == b' ': #hace que la tortuga de un giro instantáneo de 180°
        teleport('giro')
```

La siguiente función, se encarga de realizar el teletransporte absoluto a la posición y orientación iniciales al presionar la tecla R o la rotación de 180 grados al precionar la tecla espaciadora mediante un teletransporte relativo de la tortuga.
```
def teleport(tecla):
    if tecla == 'home':
        rospy.wait_for_service(teleport_ab) #Espera a que el servicio de teletransporte absoluto esté disponible.
        try:
            tel_abs_service = rospy.ServiceProxy(teleport_ab, TeleportAbsolute) #Crea un objeto de proxy del servicio de teletransporte absoluto.
            tel_abs = tel_abs_service(5.544445,5.544445,0) #Llama al servicio de teletransporte absoluto con las coordenadas especificadas 
            rospy.wait_for_service('/clear') #Espera a que el servicio de limpieza esté disponible.
            limpiar = rospy.ServiceProxy('/clear', Empty) # Crea un objeto de proxy del servicio de limpieza.
            Reset = limpiar() #Llama al servicio de limpieza para limpiar la trayectoria de la tortuga después de teletransportarla.
        except rospy.ServiceException as e:
            print(str(e))
    elif tecla == 'giro':
        rospy.wait_for_service(teleport_rel) #Espera a que el servicio de teletransporte relativo esté disponible.
        try:
            tel_rel_service = rospy.ServiceProxy(teleport_rel, TeleportRelative)#Crea un objeto de proxy del servicio de teletransporte absoluto.
            tel_rel = tel_rel_service(0,pi) #Llama al servicio de teletransporte absoluto con las orientaciones especificadas 
        except rospy.ServiceException as e:
            print(str(e))
```
Esta sección se encarga de publicar los comandos de velocidad en el tópico de velocidad de la tortuga.
```
def publicarVel(linear_x, angular_z):
    publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10) }#crea un objeto de publicador de ROS 
    message = Twist() #crea un objeto del tipo de mensaje.
    message.linear.x = linear_x #Se asigna la velocidad lineal en x
    message.angular.z = angular_z #Se asigna la velocidad angular respecto a z
    publisher.publish(message)
```

La última sección del código ejecuta un bucle principal que lee la entrada del teclado y llama a get_action() para procesarla. Muestra un mensaje de bienvenida y proporciona instrucciones sobre cómo utilizar el script. El bucle se ejecuta a una frecuencia de 10 Hz hasta que se interrumpe el nodo.
```
if __name__ == '__main__':
    
    welcome = """
    Hecho por Felipe Chaves Delgadillo y Marco Antonio Quimbay Dueñas
    --------------------------------------------------------------
    Leyendo comandos desde el teclado
    --------------------------------------------------------------
    Use AWSD para mover la tortuga
    Use 'R' para limpiar el panel y devolver a la tortuga a su posición inicial
    Use Espacio para girar la tortuga 180°
    Use 'q' para salir
    --------------------------------------------------------------
    """

    try:
        rospy.init_node('my_teleop_key') #Inicializa un nodo de ROS 
        rospy.loginfo(welcome) #Imprime el mensaje de bienvenida en la consola de ROS
        rate = rospy.Rate(10) #Establece en 10hz la velocidad a la que se ejecuta el bucle principal del script
        while not rospy.is_shutdown(): # Se inicia un bucle principal que se ejecutará continuamente hasta que se reciba una señal para apagar el nodo de ROS.
            tareaARealizar()#Obtiene y procesa la entrada del teclado
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


```


## Resultados y Análisis
Primer Programa
A continuación, se muestra el resultado de la ejecución del código dado en la guía de laboratorio:
<p align="center">
    <img src=Imagenes/primer_programa.png alt="Primer Programa " width="1000">
</p>

Subscribe ROS
Ahora veremos cual es el resultado de agregar la suscripción a los mensajes de ROS del Turtlesim:
<p align="center">
    <img src=Imagenes/Subscribe_ROS.png alt="Subscribe ROS " width="1000">
</p>

Modificar la Pose
Ahora veremos el resultado de modificar la pose del Turtle y posteriormente acceder a sus parámetros:
<p align="center">
    <img src=Imagenes/Pose_edited.png alt="Pose Edited " width="1000">
</p>

myTeleopKey
Por último, veremos el funcionamiento del código de operación de la tortuga con las teclas ASWD:
https://github.com/FelipeCh18/Lab3_Robotica/blob/main/Imagenes/Video/myTeleopKeyVideo.mkv

## Conclusiones
El laboratorio ofreció una introducción a ROS a través del uso de turtlesim, explorando temas esenciales como suscripciones, publicaciones, tópicos, mensajes, servicios, entre otros. Esto nos proporcionó una base sólida para comprender cómo estos elementos pueden ser empleados en diversas tareas y objetivos en futuros laboratorios. Además, mediante el control de una tortuga en una interfaz gráfica a través de la terminal, se establecen los fundamentos para comprender las comunicaciones entre nodos en ROS y prepararse para interacciones más complejas entre programas en diferentes lenguajes.
