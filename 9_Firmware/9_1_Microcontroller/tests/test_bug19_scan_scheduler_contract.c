#include <assert.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SOURCE_FILE "../9_1_3_C_Cpp_Code/main.cpp"
#define FALLBACK_SOURCE_FILE "/Users/ganeshpanth/PLFM_RADAR/9_Firmware/9_1_Microcontroller/9_1_3_C_Cpp_Code/main.cpp"

static char *read_file(const char *path, long *out_size)
{
    FILE *f = fopen(path, "r");
    if (!f)
        return NULL;

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    long size = ftell(f);
    if (size < 0) {
        fclose(f);
        return NULL;
    }
    rewind(f);

    char *buf = (char *)malloc((size_t)size + 1u);
    if (!buf) {
        fclose(f);
        return NULL;
    }

    size_t nread = fread(buf, 1u, (size_t)size, f);
    buf[nread] = '\0';
    fclose(f);

    if (out_size)
        *out_size = (long)nread;
    return buf;
}

static const char *skip_ws(const char *p)
{
    while (*p && isspace((unsigned char)*p))
        p++;
    return p;
}

static int parse_int_after(const char *src, const char *needle, int *out)
{
    const char *p = strstr(src, needle);
    if (!p)
        return 0;
    p += strlen(needle);
    p = skip_ws(p);
    if (sscanf(p, "%d", out) != 1)
        return 0;
    return 1;
}

static int extract_function_span(const char *src,
                                 const char *func_sig,
                                 const char **out_begin,
                                 const char **out_end)
{
    const char *sig = strstr(src, func_sig);
    if (!sig)
        return 0;

    const char *open = strchr(sig, '{');
    if (!open)
        return 0;

    int depth = 0;
    const char *p = open;
    while (*p) {
        if (*p == '{')
            depth++;
        else if (*p == '}')
            depth--;

        if (depth == 0) {
            *out_begin = open;
            *out_end = p;
            return 1;
        }
        p++;
    }
    return 0;
}

static int count_occurrences_in_span(const char *begin,
                                     const char *end,
                                     const char *needle)
{
    int count = 0;
    size_t nlen = strlen(needle);
    const char *p = begin;
    while (p && p < end) {
        const char *f = strstr(p, needle);
        if (!f || f >= end)
            break;
        count++;
        p = f + nlen;
    }
    return count;
}

static int parse_first_beam_loop_bound_in_run_seq(const char *func_begin,
                                                  const char *func_end,
                                                  char *token,
                                                  size_t token_sz)
{
    const char *bp = strstr(func_begin, "beam_pos <");
    if (!bp || bp >= func_end)
        return 0;

    const char *lt = strchr(bp, '<');
    const char *semi = strchr(bp, ';');
    if (!lt || !semi || semi >= func_end || lt > semi)
        return 0;

    lt++;
    lt = skip_ws(lt);

    size_t i = 0;
    while (lt < semi && !isspace((unsigned char)*lt) && *lt != ';' && i + 1 < token_sz) {
        token[i++] = *lt;
        lt++;
    }
    token[i] = '\0';
    return i > 0;
}

static int extract_first_beam_loop_body(const char *func_begin,
                                        const char *func_end,
                                        const char **body_begin,
                                        const char **body_end)
{
    const char *bp = strstr(func_begin, "beam_pos <");
    if (!bp || bp >= func_end)
        return 0;

    const char *f = bp;
    while (f > func_begin && *f != 'f')
        f--;

    const char *open = strchr(f, '{');
    if (!open || open >= func_end)
        return 0;

    int depth = 0;
    const char *p = open;
    while (*p && p < func_end) {
        if (*p == '{')
            depth++;
        else if (*p == '}')
            depth--;

        if (depth == 0) {
            *body_begin = open;
            *body_end = p;
            return 1;
        }
        p++;
    }
    return 0;
}

static int parse_phase_differences(const char *src, double *vals, int max_vals)
{
    const char *p = strstr(src, "const float phase_differences[31]");
    if (!p)
        return -1;

    const char *open = strchr(p, '{');
    const char *close = strchr(p, '}');
    if (!open || !close || open > close)
        return -1;

    int count = 0;
    const char *q = open + 1;
    while (q < close && count < max_vals) {
        q = skip_ws(q);
        if (q >= close)
            break;

        char *endptr = NULL;
        double v = strtod(q, &endptr);
        if (endptr == q)
            break;

        vals[count++] = v;
        q = endptr;

        while (q < close && *q != ',' && *q != '}')
            q++;
        if (*q == ',')
            q++;
    }

    return count;
}

int main(void)
{
    printf("=== Bug #19: scan scheduler + beam table contract ===\n");

    long size = 0;
    char *src = read_file(SOURCE_FILE, &size);
    if (!src)
        src = read_file(FALLBACK_SOURCE_FILE, &size);
    assert(src && "Could not open main.cpp");

    printf("  Read %ld bytes from main.cpp\n", size);

    int n_max = 0;
    assert(parse_int_after(src, "const int n_max =", &n_max) && "Could not parse n_max");
    printf("  Parsed n_max = %d\n", n_max);
    assert(n_max == 31 && "Contract: n_max must stay 31 unless architecture doc changes");

    const char *run_begin = NULL;
    const char *run_end = NULL;
    assert(extract_function_span(src, "void runRadarPulseSequence() {", &run_begin, &run_end) &&
           "Could not locate runRadarPulseSequence body");

    char loop_bound_token[32] = {0};
    assert(parse_first_beam_loop_bound_in_run_seq(run_begin, run_end,
                                                  loop_bound_token, sizeof(loop_bound_token)) &&
           "Could not parse beam loop bound");
    printf("  Beam loop bound token = '%s'\n", loop_bound_token);

    int loop_bound_ok = 0;
    if (strcmp(loop_bound_token, "n_max") == 0) {
        loop_bound_ok = 1;
    } else {
        int numeric_bound = 0;
        if (sscanf(loop_bound_token, "%d", &numeric_bound) == 1)
            loop_bound_ok = (numeric_bound == n_max);
    }
    assert(loop_bound_ok &&
           "Contract: elevation loop must iterate exactly n_max positions per azimuth");

    const char *loop_begin = NULL;
    const char *loop_end = NULL;
    assert(extract_first_beam_loop_body(run_begin, run_end, &loop_begin, &loop_end) &&
           "Could not locate beam loop body");

    int exec_calls = count_occurrences_in_span(loop_begin, loop_end, "executeChirpSequence(");
    int vector0_uses = count_occurrences_in_span(loop_begin, loop_end, "vector_0");
    printf("  executeChirpSequence calls inside beam loop = %d\n", exec_calls);
    printf("  vector_0 uses inside beam loop = %d\n", vector0_uses);

    assert(exec_calls == 1 &&
           "Contract: each elevation dwell should trigger exactly one chirp frame");
    assert(vector0_uses <= 2 &&
           "Contract: broadside vector_0 should only appear for the single center beam (TX+RX)");

    double phases[64] = {0};
    int phase_count = parse_phase_differences(src, phases, 64);
    assert(phase_count == 31 && "Contract: phase_differences must contain exactly 31 entries");

    double max_abs = 0.0;
    int zero_idx = -1;
    int strictly_descending = 1;
    for (int i = 0; i < phase_count; i++) {
        double a = fabs(phases[i]);
        if (a > max_abs)
            max_abs = a;
        if (fabs(phases[i]) < 1e-6)
            zero_idx = i;
        if (i > 0 && !(phases[i] < phases[i - 1]))
            strictly_descending = 0;
    }

    const double phase_limit_for_45deg = 180.0 * sin(M_PI / 4.0); /* ~=127.279 */
    printf("  max |phase_differences| = %.3f deg\n", max_abs);
    printf("  center zero index = %d\n", zero_idx);

    assert(zero_idx == phase_count / 2 &&
           "Contract: beam table must have broadside (0 deg) at center index");
    assert(strictly_descending &&
           "Contract: phase table should sweep monotonically from +max to -max");
    assert(max_abs <= phase_limit_for_45deg + 0.5 &&
           "Contract: +/-45deg steering implies |phase increment| <= ~127.3 deg for d=lambda/2");

    free(src);

    printf("=== Bug #19: ALL TESTS PASSED ===\n");
    return 0;
}
