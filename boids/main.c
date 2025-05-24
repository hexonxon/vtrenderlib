#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>

#include <vtrenderlib.h>

#define VT_BOID_WIDTH   20
#define VT_BOID_LENGTH  30

// Boid speed in dots per second
#define VT_BOID_SPEED   50

struct vec2f
{
    float x;
    float y;
};

struct vt_boid
{
    // position, velocity and normal vectors of a boid, normals are unit vectors (len is 1).
    struct vec2f p;
    struct vec2f v;
    struct vec2f n;

    // heading angle, in radians
    double psi;

    // current target to advance to
    struct vec2f t;
};

static struct vtr_canvas* g_vt;
static struct vt_boid g_boid;

static struct vec2f vec2f_mul_add(struct vec2f a, struct vec2f b, int scale)
{
    return (struct vec2f){a.x + b.x * scale, a.y + b.y * scale};
}

static struct vec2f vec2f_sub(struct vec2f a, struct vec2f b)
{
    return (struct vec2f){a.x - b.x, a.y - b.y};
}

static struct vec2f vec2f_unit(struct vec2f v)
{
    float m = sqrtf(v.x * v.x + v.y * v.y);
    assert(m != 0);

    struct vec2f u = (struct vec2f){v.x / m, v.y / m};
    return u;
}

static struct vec2f vec2f_normal(struct vec2f v)
{
    return (struct vec2f){-v.y, v.x};
}

static struct vec2f vec2f_rot_r(struct vec2f v, double rad)
{
    float cs = cosf(rad);
    float sn = sinf(rad);

    return (struct vec2f){
        v.x * cs - v.y * sn,
        v.x * sn + v.y * cs
    };
}

static struct vec2f vec2f_rot_d(struct vec2f v, unsigned d)
{
    return vec2f_rot_r(v, M_PI * d / 180);
}

// Map a vec2f to a vtr vertex
static struct vtr_vertex vec2f_project(struct vec2f v)
{
    struct vtr_vertex vertex;
    vertex.x = (uint16_t)(v.x + 0.5f);
    vertex.y = (uint16_t)(v.y + 0.5f);
    return vertex;
}

// Update simulation, dt is in millisecs.
static void update(double dt)
{
    struct vec2f dir;
    //dir = vec2f_unit(vec2f_sub(g_boid.t, g_boid.p));

    // Update position by integrating the roll angle (for banking) and linear speed over dt.
    double dts = dt / 1000;
    double phi = 80.0 * M_PI / 180;
    double omega = (9.81 * tanf(phi)) / VT_BOID_SPEED;
    g_boid.psi += omega * dts;
    g_boid.p.x += VT_BOID_SPEED * cosf(g_boid.psi) * dts;
    g_boid.p.y += VT_BOID_SPEED * sinf(g_boid.psi) * dts;
    g_boid.v = vec2f_rot_r(g_boid.v, omega * dts);
    g_boid.n = vec2f_normal(g_boid.v);

}

static void draw_debug_vectors(struct vt_boid* boid)
{
    struct vtr_vertex t = vec2f_project(boid->t);
    struct vtr_vertex p = vec2f_project(boid->p);
    struct vtr_vertex v = vec2f_project(vec2f_mul_add(boid->p, boid->v, VT_BOID_LENGTH + 4));
    struct vtr_vertex n = vec2f_project(vec2f_mul_add(boid->p, boid->n, VT_BOID_WIDTH + 4));

    vtr_scan_line(g_vt, p.x, p.y, v.x, v.y);
    vtr_scan_line(g_vt, p.x, p.y, n.x, n.y);
    vtr_render_dot(g_vt, t.x, t.y);
}

static void draw(void)
{
    struct vtr_vertex buf[] = {
        vec2f_project(vec2f_mul_add(g_boid.p, g_boid.n, -VT_BOID_WIDTH / 2)),
        vec2f_project(vec2f_mul_add(g_boid.p, g_boid.n, VT_BOID_WIDTH / 2)),
        vec2f_project(vec2f_mul_add(g_boid.p, g_boid.v, VT_BOID_LENGTH)),
    };

    vtr_trace_poly(g_vt, 3, buf);
    draw_debug_vectors(&g_boid);
}

static void restore_tty_attrs(void)
{
    vtr_close(g_vt);
}

static void handle_signal(int signo)
{
    if (signo == SIGWINCH) {
        vtr_set_resize_pending(g_vt);
    } else {
        restore_tty_attrs();
        signal(signo, SIG_DFL);
        raise(signo);
    }
}

int main(void)
{
    int error;

    g_vt = vtr_canvas_create(STDOUT_FILENO);
    if (!g_vt) {
        exit(EXIT_FAILURE);
    }

    atexit(restore_tty_attrs);
    signal(SIGINT, handle_signal);
    signal(SIGWINCH, handle_signal);

    error = vtr_reset(g_vt);
    if (error) {
        exit(error);
    }

    g_boid.p = (struct vec2f){50, 50};
    g_boid.v = (struct vec2f){1.0, 0.0};
    g_boid.n = vec2f_normal(g_boid.v);
    g_boid.t = (struct vec2f){vtr_xdots(g_vt) / 3 , 100};

    while (true) {
        vtr_resize(g_vt);
        update(1000.0 / 60);
        draw();
        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }

    return 0;
}
