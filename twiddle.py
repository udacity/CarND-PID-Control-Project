import re
from subprocess import check_output

def run(p):
  out = check_output(["build/pid", str(p[0]), str(p[1]), str(p[2]), "150"])
  return float(re.findall(r"[-+]?\d*\.\d+|\d+", out.decode().replace("4567", ""))[0])

def twiddle(tol=0.02):
    p = [0, 0, 0]
    dp = [1, 1, 1]
    best_err = run(p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best_error={}, p={}, dp={}".format(it, best_err, p, dp))
        for i in range(len(p)):
            p[i] += dp[i]
            err = run(p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                err = run(p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p

twiddle()