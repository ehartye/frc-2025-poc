root = ""
p = root + ".github/workflows/robot-showcase.yml"
L = open(p, encoding="utf-8").read().split("\n")
idx = [i for i, l in enumerate(L) if "e7571af8422f3dfd8310359578fdc7f67dc206e9" in l]
first = idx[0]
assert "path: builder" in "\n".join(L[first:first + 3]), L[first:first + 3]
L[first] = L[first].replace("e7571af8422f3dfd8310359578fdc7f67dc206e9 # Agent FRC 0.100.0",
                            "92ddf41933dd6026b26493c1d2c91505e9d15914 # Agent FRC 0.48.0")
open(p, "w", encoding="utf-8", newline="\n").write("\n".join(L))
g = root + "experiments/six-wheel/scripts/check_public_content.py"
s = open(g, encoding="utf-8").read()
s = s.replace("fonts" + chr(92), "fonts/")
open(g, "w", encoding="utf-8", newline="\n").write(s)
print("fixed")
