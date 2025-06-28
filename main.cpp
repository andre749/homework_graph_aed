#include <SFML/Graphics.hpp>

int main() {
    // Crear una ventana de 800x600 píxeles, con título "Ventana de prueba"
    sf::RenderWindow window(sf::VideoMode(800, 600), "Ventana de prueba");

    // Bucle principal
    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            // Cerrar la ventana si se presiona la X
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Limpiar la ventana (color negro por defecto)
        window.clear();

        // Mostrar lo que se ha dibujado
        window.display();
    }

    return 0;
}
