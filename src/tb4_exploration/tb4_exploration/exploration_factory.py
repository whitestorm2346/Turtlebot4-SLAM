from tb4_exploration.frontier_explorer import FrontierExplorer
from tb4_exploration.information_gain_explorer import InformationGainExplorer
from tb4_exploration.random_explorer import RandomExplorer


def create_explorer(exploration_method, node):
    if exploration_method == 'frontier':
        return FrontierExplorer(node)

    if exploration_method == 'information_gain':
        return InformationGainExplorer(node)

    if exploration_method == 'random':
        return RandomExplorer(node)

    raise ValueError(f'Unknown exploration method: {exploration_method}')