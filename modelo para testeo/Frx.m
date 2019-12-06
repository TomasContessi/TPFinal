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
function [val]=Frx(Vx,Vy,Vz,parametros);
    cd=parametros(14);
    masa=parametros(13);
    Vwx=parametros(9);
    Vwz=parametros(10);
    radio=parametros(12)/2;
    da=parametros(11);
    val=(cd*da*pi*(radio^2)/(2*masa))*sqrt((Vx+Vwx)^2+Vy^2+(Vz+Vwz)^2)*(Vx+Vwx);

end