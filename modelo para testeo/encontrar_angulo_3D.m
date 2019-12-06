%estimacion del angulo para disparo 3D


clear all
close all

%parametros del disparo

distancia=10e3;      %distancia al objetivo (en el suelo)(respecto a mi)(max 26km)
direccion=120;       %direccion del objetivo (respecto a mi)(entre -180° y 180°)
altura=0;           %el objetivo tiene que estar debajo de la altura de mi torreta (default 0m se puede modificar en calcular_impacto.m siendo la posicion de la torreta (respecto al puente) x0,y0,z0)

%parametros del objetivo

Velociad_objetivo_x=45;
Velociad_objetivo_y=0;
Velociad_objetivo_z=45;

%fuerzas externas

Velociad_viento=100;
direccion_viento =70;
densidad_atmosfera=1.25;

%parametros de la municion

calibre_m=0.38;
masa=800;
cd=0.295;
vel_salida=820;

%parametros del cañon

amax_alfa=22;
amin_alfa=0;

amax_beta=180;
amin_beta=-180;

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



M=100;          %cantidad maxima de iteraciones

err=distancia;
i=0;



while err >= 0.001*distancia && i<M
    
    alfa = (amax_alfa+amin_alfa)/2;
    beta = (amax_beta+amin_beta)/2;
    
    impacto=calcular_impacto(alfa,beta,parametros);
    
    distancia_impacto=sqrt(impacto(1)^2+impacto(3)^2);
    
    distancia=sqrt(impacto(5)^2+impacto(7)^2);
    
    if (distancia_impacto > distancia)
        amax_alfa=alfa;
    end
    if (distancia_impacto < distancia)
        amin_alfa=alfa;
    end
    
    angulo_impacto=atan2(impacto(1),impacto(3))*(360/(2*pi));
    direccion=atan2(impacto(5),impacto(7))*(360/(2*pi));
    posx=impacto(5);
    posz=impacto(7);
    
    if (angulo_impacto > direccion)
        amax_beta=beta;
    end
    if (angulo_impacto < direccion)
        amin_beta=beta;
    end
    
    err=sqrt((posx-impacto(1))^2 + (posz-impacto(3))^2);
    i=i+1;
end

iteraciones=i

tiempo=impacto(4)
err

%grafico solo por diversion
h=0.001;            % paso del metodo
x0=0;               % posicion inicial x
y0=0;               % posicion inicial y
z0=0;               % posicion inicial z
N=100000;           % cantidad maxima de pasos


% parametros(1)=Velociad_objetivo_x;
% parametros(2)=Velociad_objetivo_y;
% parametros(3)=Velociad_objetivo_z;
% parametros(4)=Posx;
% parametros(5)=altura;
% parametros(6)=Posz;
% parametros(9)=Velociad_viento_x;
% parametros(10)=Velociad_viento_y;
% parametros(11)=densidad_atmosfera;
% parametros(12)=calibre_m;
% parametros(13)=masa;
% parametros(14)=cd;
% parametros(15)=vel_salida;

Velociad_objetivo_x=parametros(1);
Velociad_objetivo_y=parametros(2);
Velociad_objetivo_z=parametros(3);

posx(1)=parametros(4);
posy(1)=parametros(5);
posz(1)=parametros(6);

V0=parametros(15);

%comienzo a resolver la ecuacion

Vy(1)=V0*sin(2*pi*alfa/360);
Vx(1)=V0*cos(2*pi*alfa/360)*sin(2*pi*beta/360);
Vz(1)=V0*cos(2*pi*alfa/360)*cos(2*pi*beta/360);
T(1)=0;
i=0;
X(1)=x0;
Y(1)=y0;
Z(1)=z0;
altura=0;

while altura >= posy(i+1) && i <= N
    
    i=i+1;
    %cuento el tiempo
    T(i+1)=T(i)+ h;

    %calculo los k1
    k1x=h*Fx(Vx(i),Vy(i),Vz(i),parametros);
    k1y=h*Fy(Vx(i),Vy(i),Vz(i),parametros);
    k1z=h*Fz(Vx(i),Vy(i),Vz(i),parametros);
    %calculo los k2
    k2x=h*Fx((Vx(i)+k1x/2),(Vy(i)+k1y/2),(Vz(i)+k1z/2),parametros);
    k2y=h*Fy((Vx(i)+k1x/2),(Vy(i)+k1y/2),(Vz(i)+k1z/2),parametros);
    k2z=h*Fz((Vx(i)+k1x/2),(Vy(i)+k1y/2),(Vz(i)+k1z/2),parametros);
    %calculo los k3
    k3x=h*Fx((Vx(i)+k2x/2),(Vy(i)+k2y/2),(Vz(i)+k2z/2),parametros);
    k3y=h*Fy((Vx(i)+k2x/2),(Vy(i)+k2y/2),(Vz(i)+k2z/2),parametros);
    k3z=h*Fz((Vx(i)+k2x/2),(Vy(i)+k2y/2),(Vz(i)+k2z/2),parametros);
    %calculo los k4
    k4x=h*Fx((Vx(i)+k3x),(Vy(i)+k3y),(Vz(i)+k3z),parametros);
    k4y=h*Fy((Vx(i)+k3x),(Vy(i)+k3y),(Vz(i)+k3z),parametros);
    k4z=h*Fz((Vx(i)+k3x),(Vy(i)+k3y),(Vz(i)+k3z),parametros);
    %clculo el siguiente paso
    
    Vx(i+1) =Vx(i) + 1/6*(k1x +2*k2x + 2*k3x + k4x);
    
    Vy(i+1) =Vy(i) + 1/6*(k1y +2*k2y + 2*k3y + k4y);
    
    Vz(i+1) =Vz(i) + 1/6*(k1z +2*k2z + 2*k3z + k4z);
    
    %voy integrando para ir obteniendo las posiciones
    
    Y(i+1)=(Vy(i)+Vy(i+1))*(T(i+1)-T(i))/2+Y(i);
    X(i+1)=(Vx(i)+Vx(i+1))*(T(i+1)-T(i))/2+X(i);
    Z(i+1)=(Vz(i)+Vz(i+1))*(T(i+1)-T(i))/2+Z(i);
    altura=Y(i+1);
    
    posx(i+1)=Velociad_objetivo_x*h+posx(i);
    posy(i+1)=Velociad_objetivo_y*h+posy(i);
    posz(i+1)=Velociad_objetivo_z*h+posz(i);
    
end

 grid on
 plot3(X,Z,Y,'r')
 hold on
 plot3(X(i+1),Z(i+1),Y(i+1),'*r')
 plot3(X(1),Z(1),Y(1),'*g')
 plot3(posx,posz,posy,'b')
 plot3(posx(i+1),posz(i+1),posy(i+1),'*b')
 grid on