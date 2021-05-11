from gym.envs.registration import register

register(
    id='ihm-v0',
    entry_point='gym_env_files.envs:IHMBulletEnv',
    kwargs={},
    max_episode_steps = 200
)