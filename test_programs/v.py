ls = [1, 2, 3, 4]
pt = [8, 9, 10, 11]
l = [(3, 9), (10, 12), (5, 6)]
new = []
for i in ls:
    try:
        new.index(i)
    except ValueError:
        new.append(i)
        print(new)

p = (5, 6)
if 1 in range(0, 10):
    print("True")
if 11 not in range(0, 9):
    print("False")
try:
    a = l.index(p)
    print(a)
except IndexError:
    print("Not Working")