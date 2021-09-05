""" Implements commonly used utilities.
"""

def get_stem(path):
    """ Returns a stem of the given path.

    Args:
        path: The path from which to extract the stem.
    Returns:
        The stem.
    """
    return path.basename[:-len(path.extension) - 1]
