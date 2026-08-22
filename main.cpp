#include "flock.hpp"

#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

int main()
{
    int number_boids{};

    double d{};
    double ds{};
    double s{};
    double a{};
    double c{};

    double max_speed{};

    double Lx{};
    double Ly{};
    double Lz{};

    double dt{};
    int steps{};

    std::cout << "Boid's number: ";
    std::cin >> number_boids;

    std::cout << "Distanza di percezione d: "; //da rendere tutto in inglese
    std::cin >> d;

    std::cout << "Distanza di separazione ds: ";
    std::cin >> ds;

    std::cout << "Fattore di separazione s: ";
    std::cin >> s;

    std::cout << "Fattore di allineamento a: ";
    std::cin >> a;

    std::cout << "Fattore di coesione c: ";
    std::cin >> c;

    std::cout << "Velocita massima: ";
    std::cin >> max_speed;

    std::cout << "Dimensione Lx: ";
    std::cin >> Lx;

    std::cout << "Dimensione Ly: ";
    std::cin >> Ly;

    std::cout << "Dimensione Lz: ";
    std::cin >> Lz;

    std::cout << "Intervallo temporale dt: "; //frequenza temporale con cui viene analizzato il sistema
    std::cin >> dt;

    std::cout << "Numero di passi temporali: "; //quante volte si aggiorna il sistema
    std::cin >> steps;

    // CONTROLLO INPUT

    if (std::cin.fail()) {
        std::cerr << "Errore: input non valido.\n";
        return 1;
    }

    if (number_boids < 2) {
        std::cerr << "Errore: servono almeno due boid.\n";
        return 1; //programma terminato per un errore
    }

    if (d <= 0. || s < 0. || a < 0. || a >= 1. || c < 0.) {
    std::cerr << "Errore: Almeno uno dei parametri d, s, a, c non validi.\n";
    return 1;
    }

    if (ds <= 0. || ds >= d) {
        std::cerr << "Errore: deve essere 0 < ds < d.\n";
        return 1;
    }

    

    if (max_speed <= 0.) {
        std::cerr << "Errore: la velocita massima deve essere positiva.\n";
        return 1;
    }

    if (Lx <= 0. || Ly <= 0. || Lz <= 0.) {
        std::cerr << "Errore: le dimensioni dello spazio devono essere positive.\n";
        return 1;
    }

    if (dt <= 0. || steps <= 0) {
        std::cerr << "Errore: dt e numero di passi devono essere positivi.\n";
        return 1;
    }

    // GENERATORE DI NUMERI CASUALI

    std::random_device rd;
    std::default_random_engine generator{rd()};

    std::uniform_real_distribution<double> random_x{-Lx / 2., Lx / 2.};
    std::uniform_real_distribution<double> random_y{-Ly / 2., Ly / 2.};
    std::uniform_real_distribution<double> random_z{-Lz / 2., Lz / 2.};

    std::uniform_real_distribution<double> random_velocity{-max_speed / std::sqrt(3.), max_speed / std::sqrt(3.)}; //così il modulo max dell'oggetto è pari a max_speed

    // CREAZIONE DEI BOID

    std::vector<boids::Boid> initial_boids;
    initial_boids.reserve(number_boids);

    for (int i = 0; i < number_boids; ++i) {

        boids::Vector3 position{random_x(generator), random_y(generator), random_z(generator)
        };

        boids::Vector3 velocity{random_velocity(generator), random_velocity(generator), random_velocity(generator)
        }; // Evita che la velocita iniziale superi max_speed
        initial_boids.emplace_back(position, velocity); //serve ad aggiungere i piccioni al vettore di piccioni da sinistra(?)
    }

    // CREAZIONE DELLO STORMO

    boids::Flock flock{ initial_boids, d, ds, s, a, c, max_speed, Lx, Ly, Lz
    };

    // SIMULAZIONE

    std::cout << std::fixed << std::setprecision(4); //usa la notazione decimale normale, e non scientifica (aka e04 etc.), ed i numeri stampati hanno 4 cifre dopo la virgola

    std::cout << "\n"; //stampo prima perché così associo al tempo 0 la generazione iniziale 
    std::cout << "time "
              << "mean_distance "
              << "distance_stddev "
              << "mean_speed "
              << "speed_stddev\n";

    for (int step = 0; step <= steps; ++step) {

        double const time = step * dt;

        std::cout << time << ' '
                  << flock.mean_distance() << ' '
                  << flock.distance_stddev() << ' '
                  << flock.mean_speed() << ' '
                  << flock.speed_stddev() << '\n';

        if (step < steps) {
            flock.update(dt);
        }
    }

    return 0;
}