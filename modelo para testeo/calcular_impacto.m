function [impacto]=calcular_impacto(alfa,beta,parametros);
%resolucion por runge kutta 4
%
%k1=h*f(xn,yn)
%k2=h*f(xn+h/2,yn+k1/2)
%k3=h*f(xn+h/2,yn+k2/2)
%k4=h*f(xn+h,yn+k3)
%
%yn+1 =yn + 1/6*(k1 +2*k2 + 2*k3 + k4)


%seteo(parametros de la atmosfera y la municion se setean por separado)

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

impacto(1)=X(i+1);
impacto(2)=Y(i+1);
impacto(3)=Z(i+1);
impacto(4)=T(i+1);

impacto(5)=posx(i+1);
impacto(6)=posy(i+1);
impacto(7)=posz(i+1);

velocidad_final=sqrt(Vx(i+1)^2+Vy(i+1)^2+Vz(i+1)^2)
angulo_impacto=atan2(Vy(i+1),sqrt(Vx(i+1)^2+Vz(i+1)^2))*(360/(2*pi))


end


