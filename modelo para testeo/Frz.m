function [val]=Frz(Vx,Vy,Vz,parametros);
    cd=parametros(14);
    masa=parametros(13);
    Vwx=parametros(9);
    Vwz=parametros(10);
    radio=parametros(12)/2;
    da=parametros(11);
    val=(cd*da*pi*(radio^2)/(2*masa))*sqrt((Vx+Vwx)^2+Vy^2+(Vz+Vwz)^2)*(Vz+Vwz);

end