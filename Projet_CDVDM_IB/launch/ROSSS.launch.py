import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node principale qui gère l'activation/désactivation des autres
    controle = Node(
        package='Projet_CDVDM_IB',
        executable='controle',
        name='controle',  # Nom unique
        output='screen',
        # Si cette node s'arrête, le système s'arrête (optionnel)
        on_exit=lambda event: Shutdown() if event.rc != 0 else None,
    )

    # Nodes gérées par 'controle' (pas de Shutdown automatique)
    linefollow = Node(
        package='Projet_CDVDM_IB',
        executable='linefollow',
        name='linefollow',
        output='screen',
        # Pas de on_exit=Shutdown() pour permettre une gestion dynamique
    )

    obstacleavoidance = Node(
        package='Projet_CDVDM_IB',
        executable='obstacleavoidance',
        name='obstacleavoidance', 
        output='screen',
    )

    corridornavigation = Node(
        package='Projet_CDVDM_IB',
        executable='corridornavigation',
        name='corridornavigation',
        output='screen',
    )

    goal = Node(
        package='Projet_CDVDM_IB',
        executable='goal',
        name='goal',  
        output='screen',
    )

    return LaunchDescription([
        linefollow,
        obstacleavoidance,
        corridornavigation,
        goal,
        controle,
    ])