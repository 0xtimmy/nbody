
#ifndef PHYSICS_H
#define PHYSICS_H

#include<stdio.h>
#include<math.h>

#define GRAV_CONST 0.6674
#define MIN_MASS 1.0
#define MAX_MASS 2000.0

struct vec {
    double x;
    double y;

    vec operator+(const vec& other) const {
        return {x + other.x, y + other.y};
    }
    vec operator*(const vec& other) const {
        return {x * other.x, y * other.y};
    }

    vec operator*(const int& other) const {
        return {x * (double)(other), y * (double)(other)};
    }
    vec operator/(const int& other) const {
        return {x / (double)(other), y / (double)(other)};
    }

    vec operator*(const double& other) const {
        return {x * other, y * other};
    }
    vec operator/(const double& other) const {
        return {x / other, y / other};
    }

};

double dist(vec *a, vec *b);

struct body {
    int id;
    double mass;
    vec pos;
    vec v;
};

struct environment {
    body* bodies;
    int n;
    double t;

    int width;
    int height;

    char *init_window() {
        char *window = (char *)malloc(sizeof(char) * (1 + ((width+3) * (height+2))));
        for(int x = 0; x < width+2; x++) {
            if (x == 0 || x == width+1) window[x] = '+';
            else window[x] = '-';
        }
        window[width+2] = '\n';
        for(int y = 1; y < height+1; y++) {
            window[y*(width+3)] = '|';
            for(int x = 1; x < width+1; x++) {
                window[x + y*(width+3)] = ' ';
            }
            window[width+1 + y*(width+3)] = '|';
            window[width+2 + y*(width+3)] = '\n';
        }
        for(int x = 0; x < width+2; x++) {
            if (x == 0 || x == width+1) window[x+(height+1)*(width+3)] = '+';
            else window[x+(height+1)*(width+3)] = '-';
        }
        window[width+2 + (height+1)*(width+3)] = '\n';
        window[width+3 + (height+1)*(width+3)] = '\0';
        return window;
    }

    void render(char *window) {
        for(int y = 1; y < height+1; y++) {
            for(int x = 1; x < width+1; x++) {
                window[x + y*(width+3)] = ' ';
            }
        }
        double max_dim = 0;
        for(int i = 0; i < n; i++) {
            if(fabs(bodies[i].pos.x) > max_dim) max_dim = fabs(bodies[i].pos.x);
            if(fabs(bodies[i].pos.y) > max_dim) max_dim = fabs(bodies[i].pos.y);
        }
        max_dim *= 1.1;
        for(int i = 0; i < n; i++) {
            int x_norm = (int)floor(((bodies[i].pos.x / (2*max_dim)) + 0.5) * width)+1;
            int y_norm = (int)floor(((bodies[i].pos.y / (2*max_dim)) + 0.5) * height)+1;
            window[x_norm + y_norm*(width+3)] = '0' + bodies[i].id;
        }
    }

    bool collision() {
        for(int i = 0; i < n; i++) {
            for(int j = i+1; j < n; j++) {
                if(dist(&(bodies[i].pos), &(bodies[j].pos)) < 0.001) {
                    return true;
                }
            }
        }
        return false;
    }
};

vec randpos();
vec force(body *bdy, environment *e);
vec _force(body *a, body *b);
environment init_environment(int n);
void pprint_environment (environment *env);
void pprint_body (body *x);

vec randvec() {
    vec out = { 
        (((double)(rand()%2000))/100-10), 
        (((double)(rand()%2000))/100-10)
    };
    return out;
}

vec force(body *bdy, environment *e) {
    vec force = { 0.0, 0.0 };
    for (int i = 0; i < e->n; i++) {
        if(bdy != (e->bodies + i)) {
            force = force + _force(bdy, &(e->bodies[i]));
        }
    }
    return force;
}

double dist(vec *a, vec *b) {
    double delta_x = a->x - b->x;
    double delta_y = a->y - b->y;
    return sqrt(delta_x*delta_x + delta_y*delta_y);
}

vec _force(body *a, body *b) {
    double delta_x = a->pos.x - b->pos.x;
    double delta_y = a->pos.y - b->pos.y;
    double r = dist(&(a->pos), &(b->pos));
    double mag = fabs(GRAV_CONST * (a->mass * b->mass) / exp2f(r));
    //fprintf(stderr, "mag: %f, r: %f, delta_x: %f, delta_y: %f\n", mag, r, delta_x, delta_y);
    vec force = { 
        -mag*delta_x/r, 
        -mag*delta_y/r,
    };
    //fprintf(stderr, "%d <-> %d = <%f, %f>\n", a->id, b->id, force.x, force.y);
    return force;
}

environment init_environment(int n) {
    environment env;
    env.n = n;
    env.t = 0;
    env.width = 100;
    env.height = 40;
    env.bodies = new body [n];

    for (int i = 0; i < n; i++) {
        env.bodies[i] = { i, ((double)(rand()%(1000))*(MAX_MASS-MIN_MASS)/1000.0 + MIN_MASS), randvec()/10, {0, 0}};
    }
    return env;
}

void step(environment *in, environment *out, double delta_t) {
    for (int i = 0; i < in->n; i++) {
        vec a = force((&in->bodies[i]), in) / in->bodies[i].mass;
        out->bodies[i].v = in->bodies[i].v + a * delta_t;
        out->bodies[i].pos = in->bodies[i].pos + out->bodies[i].v;
    }
}

void pprint_environment(environment *env) {
    fprintf(stderr, "=== Environment @ %f =====\n", env->t);
    for (int i = 0; i < env->n; i++) {
        pprint_body(&(env->bodies[i]));
    }
}

void pprint_body(body *x) {
    fprintf(
        stderr, 
        "--- Body: %d -----\nMass: %f\nAt: <%f, %f>\nGoing: <%f, %f>\n------------------\n", 
        x->id, x->mass, 
        x->pos.x, x->pos.y,
        x->v.x, x->v.y
    );
}

#endif