	software, modelos 3d para hardware y programas de testeo para el tp final


--------------------------------------IMPORTANTE------------------------------------
para que el codigo corra con el firmware v2 de la EDU CIAA se debe modificar una libreria nativa
comentando el while (1) de la funcion HardFault_Handler() en la linea 458 del archivo cr_startup_lpc43xx.c

(es facil de encontrar, se corre el programa en debugg, se va a trabar, se frena el debugg y la linea donde se trabo es ese while 1)
