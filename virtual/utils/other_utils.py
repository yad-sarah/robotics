from robosuite.wrappers import GymWrapper
import robosuite as suite

def make_env_fn(env_name="Lift", robots=["Panda"]):
    def _init():
        print('creating env ...')
        env = suite.make(
            env_name=env_name,
            robots=robots,
            has_renderer=False,
            render_camera="frontview",
            has_offscreen_renderer=False,
            use_camera_obs=False,  # Not using camera observations
        )
        env = GymWrapper(env)
        return env
    return _init