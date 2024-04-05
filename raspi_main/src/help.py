class foo():
    def __init__(self,var) -> None:
        self.abc = var

a = [foo(14),foo(15)]

print(a.remove(1))