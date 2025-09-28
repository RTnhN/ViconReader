from . import Interpolation


class Akmia(Interpolation.Interpolation):
    def __init__(self, data):
        super(Akmia, self).__init__(data)

    def interpolate(self, verbose):
        for key, value in self.data.items():  # For every subject in the data...
            for sub_key, sub_value in value.items():
                if "Magnitude( X )" not in value.keys() and "Count" not in value.keys():
                    sub_value["data"] = Interpolation.akmia(
                        sub_value, verbose, "Trajectory", sub_key, key
                    )
                    if verbose:
                        print(
                            "Interpolating missing values in field "
                            + sub_key
                            + ", in subject "
                            + key
                            + ", in category Trajectories..."
                        )
