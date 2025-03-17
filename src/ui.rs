use bevy::prelude::*;
// use crate::{FPSNode, MyTimer};

#[derive(Resource, Deref, DerefMut)]
pub struct MyTimer(Timer);


#[derive(Component)]
pub struct FPSNode;

impl MyTimer {
    // type Target = Timer;
    fn new() -> Self {
        MyTimer (Timer::from_seconds(1., TimerMode::Repeating))
    }

}
pub fn update_ui(timer: ResMut<MyTimer>, mut query: Query<&mut Text, With<FPSNode>>, time: Res<Time<Real>>) {
    let mut s= query.single_mut();
    // let t = t.into_inner();

    let timer = timer.into_inner();
    let delta = time.into_inner().delta();
    timer.tick(delta);

    if timer.finished() {
        let fps = 1. / delta.as_secs_f32();

        s.clear();
        s.push_str(format!("FPS {fps:.2}").as_str());
    }


    // if t.el

}
pub fn setup_ui(mut commands: Commands) {
    commands.insert_resource(MyTimer::new());
    commands.spawn((
        Node {
            display: Display::Flex,
            flex_direction: FlexDirection::Row,
            width: Val::Percent(100.), ..default()
        },
        // HIGH_RES_LAYERS,
    ))
        .with_children(
            |builder| {
                builder.spawn((
                    Text::new("0"),
                    FPSNode,
                    TextFont {
                        font_size: 20.0,
                        ..default()
                    },
                    TextColor::WHITE,
                    // HIGH_RES_LAYERS

                ));
            }
        );
}


