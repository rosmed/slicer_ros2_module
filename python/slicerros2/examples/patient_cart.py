"""Load a running dVRK patient cart. Implementation lives in slicerros2.dvrk."""

_cart = None


def run(*, include_SUJ=False, include_ECM=True, fixed_frame="world", fit_view=True):
    from slicerros2.dvrk import PatientCart
    global _cart
    _cart = PatientCart(include_SUJ=include_SUJ, include_ECM=include_ECM, fixed_frame=fixed_frame)
    return _cart.load(wait=True, fit_view=fit_view)


def cleanup():
    global _cart
    if _cart is not None:
        _cart.unload()
        _cart = None
