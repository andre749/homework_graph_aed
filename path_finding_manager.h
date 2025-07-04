//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <set>
#include <queue>
#include <algorithm>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    BestFirstSearch,
    Dijkstra,
    AStar
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist < other.dist;
        }
    };
    double heuristic(Node* a, Node* b) {
        return std::hypot(a->coord.x - b->coord.x, a->coord.y - b->coord.y);
    }

    void dijkstra(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> distance;

        for (auto& node: graph.nodes) {
            distance[node.second] = std::numeric_limits<double>::infinity();
        }
        distance[src] = 0.0;

        using Pair = std::pair<double, Node*>;
        std::priority_queue<Pair, std::vector<Pair>, std::greater<>> pq;
        pq.emplace(0.0, src);

        while (!pq.empty()) {
            auto [current_distance, current_node] = pq.top();
            pq.pop();

            if (current_node == dest) break;

            if (current_distance > distance[current_node]) continue;

            for (Edge* edge : current_node->edges) {
                Node *neighbor;
                if(edge->src == current_node){
                    neighbor = edge->dest;
                } else{
                    neighbor = edge->src;;
                }

                if (edge->one_way && edge->src != current_node) continue;

                double new_distance = distance[current_node] + edge->getCost();

                if (new_distance < distance[neighbor]) {
                    distance[neighbor] = new_distance;
                    parent[neighbor] = current_node;
                    pq.emplace(new_distance, neighbor);

                    visited_edges.emplace_back(
                            current_node->coord,
                            neighbor->coord,
                            sf::Color(248, 248, 248),
                            1.0f
                    );

                    render();
                }
            }
        }
        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> g_score;
        std::unordered_map<Node *, double> f_score;

        for (auto& node: graph.nodes) {
            g_score[node.second] = std::numeric_limits<double>::infinity();
            f_score[node.second] = std::numeric_limits<double>::infinity();
        }
        g_score[src] = 0.0;
        f_score[src] = heuristic(src, dest);

        using Pair = std::pair<double, Node*>;
        std::priority_queue<Pair, std::vector<Pair>, std::greater<>> pq;
        pq.emplace(f_score[src], src);

        while (!pq.empty()) {
            auto [current_f, current_node] = pq.top();
            pq.pop();

            if (current_node == dest) break;
            if (current_f > f_score[current_node]) continue;

            for (Edge* edge : current_node->edges) {
                Node *neighbor = (edge->src == current_node ? edge->dest : edge->src);
                if (edge->one_way && edge->src != current_node) continue;

                double tentative_g = g_score[current_node] + edge->getCost();
                if (tentative_g < g_score[neighbor]) {
                    parent[neighbor] = current_node;
                    g_score[neighbor] = tentative_g;
                    f_score[neighbor] = tentative_g + heuristic(neighbor, dest);
                    pq.emplace(f_score[neighbor], neighbor);

                    visited_edges.emplace_back(
                            current_node->coord,
                            neighbor->coord,
                            sf::Color(150, 150, 150),
                            1.0f
                    );
                    render();
                }
            }
        }
        set_final_path(parent);
    }




    void Best_First_Search(Graph &graph) {
        std::unordered_map<Node*, Node*> parent;
        std::set<Node*> visited;

        using Pair = std::pair<double, Node*>;
        std::priority_queue<Pair, std::vector<Pair>, std::greater<>> pq;
        pq.emplace(heuristic(src, dest), src);

        while (!pq.empty()) {
            auto [h, current] = pq.top();
            pq.pop();

            if (current == dest) break;
            if (visited.count(current)) continue;

            visited.insert(current);

            for (Edge* edge : current->edges) {
                Node *neighbor;
                if (edge->src == current)
                    neighbor = edge->dest;
                else if (!edge->one_way)
                    neighbor = edge->src;
                else
                    continue;

                if (!visited.count(neighbor)) {
                    parent[neighbor] = current;
                    pq.emplace(heuristic(neighbor, dest), neighbor);

                    visited_edges.emplace_back(
                            current->coord,
                            neighbor->coord,
                            sf::Color(150, 150, 150),
                            1.0f
                    );

                    render();
                }
            }
        }

        set_final_path(parent);
    }



    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        sf::sleep(sf::milliseconds(10));

        sf::RenderWindow& window = window_manager->get_window();

       //window.clear(sf::Color::White);

        // Dibujar las aristas visitadas (en color claro o intermedio)
        for (sfLine& edge : visited_edges) {
            edge.draw(window, sf::RenderStates::Default);
        }

        // Dibujar el camino final (por encima, para destacarlo)
        for (sfLine& segment : path) {
            segment.draw(window, sf::RenderStates::Default);
        }

        // Dibujar el nodo de inicio (en un color distinto si lo deseas)
        if (src != nullptr) {
            src->draw(window);
        }

        // Dibujar el nodo de destino
        if (dest != nullptr) {
            dest->draw(window);
        }

        window.display();
    }


    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        path.clear();

        // Reconstruir el camino desde dest hasta src
        Node* current = dest;
        while (current && parent.count(current) && parent[current] != nullptr) {
            Node* prev = parent[current];
            // Crea la línea entre prev->coord y current->coord,
            // usa un color llamativo y un grosor mayor para destacar el camino
            path.emplace_back(
                    prev->coord,
                    current->coord,
                    sf::Color::Red,     // color del camino final
                    2.0f                // grosor del camino final
            );
            current = prev;
        }

        // Ahora el vector está de atrás hacia adelante; lo invertimos
        std::reverse(path.begin(), path.end());
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        // Si no hay origen o destino, nada que hacer
        if (src == nullptr || dest == nullptr) {
            return;
        }

        // 1) Limpiar estado previo
        path.clear();
        visited_edges.clear();

        // 2) Ejecutar el algoritmo seleccionado
        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            case BestFirstSearch:
                Best_First_Search(graph);
                break;
            default:
                // Ningún algoritmo válido, salimos
                return;
        }

        // 3) Mostrar el camino completo al final
        render();
    }


    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
