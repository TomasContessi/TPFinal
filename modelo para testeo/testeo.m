%prueva para ver si le pegue

clear all
close all


alfa=5.15625;   %lo que me tiro el programa
beta=0;         %lo que me tiro el programa

%parametros del disparo

distancia=10000;        %distancia al objetivo (en el suelo)(respecto a mi)(max 26km)
direccion=0;            %direccion del objetivo (respecto a mi)(entre -180° y 180°)
altura=0;               %el objetivo tiene que estar debajo de la altura de mi torreta (default 0m se puede modificar en calcular_impacto.m siendo la posicion de la torreta (respecto al puente) x0,y0,z0)

%parametros del objetivo

Velociad_objetivo_x=0;
Velociad_objetivo_y=0;
Velociad_objetivo_z=0;

%fuerzas externas

Velociad_viento=0;
direccion_viento =90;
densidad_atmosfera=1.25;

%parametros de la municion

calibre_m=0.38;
masa=800;
cd=0.295;
vel_salida=820;



%programa

Velociad_viento_x=Velociad_viento*sin(2*pi*direccion_viento/360);
Velociad_viento_y=Velociad_viento*cos(2*pi*direccion_viento/360);

posx=distancia*sin(2*pi*direccion/360);
posz=distancia*cos(2*pi*direccion/360);


parametros(1)=Velociad_objetivo_x;
parametros(2)=Velociad_objetivo_y;
parametros(3)=Velociad_objetivo_z;
parametros(4)=posx;
parametros(5)=altura;
parametros(6)=posz;
parametros(9)=Velociad_viento_x;
parametros(10)=Velociad_viento_y;
parametros(11)=densidad_atmosfera;
parametros(12)=calibre_m;
parametros(13)=masa;
parametros(14)=cd;
parametros(15)=vel_salida;


impacto=calcular_impacto(alfa,beta,parametros);


err=sqrt((impacto(5)-impacto(1))^2 + (impacto(7)-impacto(3))^2)