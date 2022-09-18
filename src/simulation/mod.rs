
#[cfg(not(feature = "oop"))]
mod datadriven;
#[cfg(not(feature="oop"))]
pub use datadriven::*;


#[cfg(feature = "oop")]
mod oop;
#[cfg(feature = "oop")]
pub use oop::*;
