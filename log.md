Notas para la reunión del lunes 28 de junio 2021: 

1. Revisamos documentación del Micrófono y de los protocolos. El micrófono como tal no es I2S, es un MEMS que devuelve PDM. Se utiliza I2S o SPI porque son protocolos síncronos que permiten iniciar al micrófono, pero se puede usar cualquiera de estos dos protocolos. 

2. De acuerdo a las observaciones del ruteo del manual de Ophyra, el WS no está ruteada a ningún dispositivo. Por otro lado, Las conexiones físicas a SPI son las siguientes: PB13 (SPI2_SCK) está conectado a MIC_CLK, PC3 (SPI2_MOSI) está conectado a DOUT y PB12(SPI2_NSS) está conectado a W/S (esto último entra en conflicto).

3. ¿Cómo se echó a andar el micrófono? Si no hay una librería de I2S y sin tener conexiones completas de SPI (falta MISO), ¿de qué manera se hicieron pruebas del micrófono? Una posibilidad de que el audio haya resultado cortado en las primeras pruebas, es que los datos se convierten una vez están listos los datos y eso evita tener audio continuo, se requiere funcionalidad de DMA para tener audio continuo. 

4. Existe un Pull Request desarrollado oficialmente, que ofrecen funcionalidad de I2S. Sin embargo, se requeriría conocimiento de cómo compilar el firmware con esta característica. https://www.youtube.com/watch?v=UXt27kOokh0