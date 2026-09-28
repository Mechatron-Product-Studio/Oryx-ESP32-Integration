# Cómo contribuir

¡Gracias por querer mejorar el firmware de Oryx Signal Studio!

1. Haz un fork del repositorio y crea una rama para tu cambio.
2. Edita el sketch en su carpeta (`oryx_signal_esp32/` o `oryx_signal_sd/`).
   Cada uno es un solo archivo `.ino` a propósito.
3. Comprueba que compile con el core **esp32 by Espressif Systems** 3.x y la
   placa **ESP32 Dev Module**. El firmware SD usa **U8g2**, **ESP32Encoder** y
   **Button2**.
4. Abre un pull request contra `main` contando qué cambia y cómo lo probaste
   (idealmente en una placa real).

Cada pull request se compila solo. Si es tu primera contribución, un
responsable tiene que aprobar esa compilación antes de que corra.

Un responsable revisa y acepta los cambios. Al entrar a `main`, el código se
muestra en la web y los binarios nuevos quedan disponibles para cargarse desde
el navegador, así que los cambios en el protocolo con la app necesitan
coordinarse con ella.

No hace falta tocar `.github/` para contribuir: los cambios ahí los revisa
siempre un responsable.
