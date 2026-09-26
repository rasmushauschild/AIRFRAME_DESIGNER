"""Validate UI nose-lift selections and translate them to packed simulation outputs."""
def packed_selection(rotors, indices, editor=False, required=True):
    if not 1 <= sum(r.enabled for r in rotors) <= 12:
        raise ValueError("SITL requires 1–12 active motors")
    mapping = {i: j for j, (i, r) in enumerate((i, r) for i, r in enumerate(rotors) if r.enabled)}
    count = len(rotors) if editor else len(mapping)
    if not isinstance(indices, list) or any(type(i) is not int or not 0 <= i < count for i in indices):
        raise ValueError("Invalid nose-lift motor selection; select the motors again")
    if len(indices) != len(set(indices)):
        raise ValueError("A motor can only be selected once")
    result = [mapping[i] for i in indices if i in mapping] if editor else list(indices)
    if required and not result:
        raise ValueError("Choose at least one enabled motor to lift the nose")
    return result
