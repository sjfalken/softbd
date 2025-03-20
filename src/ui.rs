use bevy::prelude::*;
// use crate::{FPSNode, MyTimer};

#[derive(Resource)]
pub struct MyTimer {
    timer: Timer,
    frame_count: usize,
}


#[derive(Component)]
pub struct FPSNode;


impl MyTimer {
    // type Target = Timer;
    fn new() -> Self {
        MyTimer{ timer: Timer::from_seconds(1., TimerMode::Repeating), frame_count: 0 }
    }

}
pub fn update_ui(timer: ResMut<MyTimer>, mut query: Query<&mut Text, With<FPSNode>>, time: Res<Time<Real>>) {
    let mut s= query.single_mut();
    // let t = t.into_inner();

    let mytimer = timer.into_inner();
    let delta = time.into_inner().delta();
    mytimer.timer.tick(delta);
    mytimer.frame_count += 1;

    if mytimer.timer.finished() {
        let fps = mytimer.frame_count;
        mytimer.frame_count = 0;

        s.clear();
        s.push_str(format!("FPS {fps}").as_str());
    }


    // if t.el

}
pub fn setup_ui(mut commands: Commands) {
    commands.insert_resource(MyTimer::new());
    commands.spawn((
        Node {
            display: Display::Flex,
            flex_direction: FlexDirection::Row,
            width: Val::Percent(100.), 
            padding: UiRect::all(Val::VMin(2.)),
            ..default()
        },
        // HIGH_RES_LAYERS,
    ))
        .with_children(
            |builder| {
                builder.spawn((
                    Text::new("FPS [Calculating]"),
                    FPSNode,
                    TextFont {
                        font_size: 1.0, // will be scaled later
                        ..default()
                    },
                    TextColor::WHITE,
                ));
            }
        );
}


