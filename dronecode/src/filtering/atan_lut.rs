
use super::filter_util::{Fix32_14,Fix16_7};

pub struct Lut{
    prev_index : u16,
    prev_y : u16,
    prev_angle : u8
}

impl Lut{ 
    const LUT:[(u8,u16);199] = [
		(0,0),
		(1,1),
		(2,2),
		(3,3),
		(4,4),
		(5,5),
		(6,6),
		(7,7),
		(8,8),
		(9,9),
		(10,10),
		(11,11),
		(12,12),
		(13,13),
		(14,14),
		(15,15),
		(16,16),
		(17,17),
		(18,18),
		(19,19),
		(20,20),
		(21,21),
		(22,22),
		(23,23),
		(24,24),
		(25,25),
		(26,26),
		(27,27),
		(28,28),
		(29,29),
		(30,30),
		(31,31),
		(32,32),
		(33,33),
		(34,34),
		(35,35),
		(36,37),
		(37,38),
		(38,39),
		(39,40),
		(40,41),
		(41,42),
		(42,43),
		(43,44),
		(44,45),
		(45,46),
		(46,48),
		(47,49),
		(48,50),
		(49,51),
		(50,52),
		(51,53),
		(52,55),
		(53,56),
		(54,57),
		(55,58),
		(56,59),
		(57,61),
		(58,62),
		(59,63),
		(60,64),
		(61,66),
		(62,67),
		(63,68),
		(64,69),
		(65,71),
		(66,72),
		(67,73),
		(68,75),
		(69,76),
		(70,77),
		(71,79),
		(72,80),
		(73,82),
		(74,83),
		(75,85),
		(76,86),
		(77,87),
		(78,89),
		(79,90),
		(80,92),
		(81,94),
		(82,95),
		(83,97),
		(84,98),
		(85,100),
		(86,101),
		(87,103),
		(88,105),
		(89,106),
		(90,108),
		(91,110),
		(92,112),
		(93,113),
		(94,115),
		(95,117),
		(96,119),
		(97,121),
		(98,123),
		(99,125),
		(100,127),
		(101,129),
		(102,131),
		(103,133),
		(104,135),
		(105,137),
		(106,139),
		(107,141),
		(108,143),
		(109,146),
		(110,148),
		(111,150),
		(112,153),
		(113,155),
		(114,158),
		(115,160),
		(116,163),
		(117,166),
		(118,168),
		(119,171),
		(120,174),
		(121,177),
		(122,180),
		(123,183),
		(124,186),
		(125,189),
		(126,193),
		(127,195),
		(128,199),
		(129,202),
		(130,206),
		(131,210),
		(132,213),
		(133,217),
		(134,221),
		(135,225),
		(136,229),
		(137,234),
		(138,238),
		(139,243),
		(140,247),
		(141,252),
		(142,257),
		(143,263),
		(144,268),
		(145,273),
		(146,279),
		(147,285),
		(148,290),
		(149,297),
		(150,303),
		(151,310),
		(152,317),
		(153,324),
		(154,332),
		(155,340),
		(156,348),
		(157,357),
		(158,366),
		(159,375),
		(160,385),
		(161,396),
		(162,407),
		(163,418),
		(164,430),
		(165,443),
		(166,456),
		(167,471),
		(168,486),
		(169,500),
		(170,517),
		(171,535),
		(172,554),
		(173,574),
		(174,597),
		(175,620),
		(176,646),
		(177,674),
		(178,704),
		(179,737),
		(180,773),
		(181,812),
		(182,856),
		(183,905),
		(184,959),
		(185,1020),
		(186,1089),
		(187,1168),
		(188,1260),
		(189,1366),
		(190,1477),
		(191,1626),
		(192,1807),
		(193,2034),
		(194,2325),
		(195,2714),
		(196,3257),
		(197,4073),
		(198,5431)
	];
    pub fn new() -> Lut{
        Lut { 
            prev_index: 0, 
            prev_y: 0, 
            prev_angle: 0 
        }
    }


    pub fn search_angle(&mut self, y_value: u16) -> u8 {
        // If the current y is the same as the previous, return the stored angle.
        if self.prev_y == y_value {
            return self.prev_angle;
        }
    
        // Alias for easier access.
        let lut = &Lut::LUT;
        let lut_len = lut.len();
    
        // Depending on whether y_value is larger or smaller, we traverse forward or backward.
        if self.prev_y < y_value {
            // Forward traversal.
            // Start at self.prev_index and initialize prev to that entry.
            let mut index = self.prev_index;
            let mut prev = lut[index as usize];
    
            // Loop until we reach the end.
            while index < (lut_len - 1 ) as u16 {
                index += 1;
                let current = lut[index as usize];
                if current.1 == y_value {
                    self.prev_angle = current.0;
                    self.prev_y = current.1;
                    self.prev_index = index;
                    return current.0;
                } else if current.1 > y_value {
                    // Interpolate between `prev` and `current`
                    let angle = self.mean(current.0, prev.0);
                    return angle;
                }
                // Update prev for the next iteration.
                prev = current;
            }
            // If y_value is larger than all LUT entries, return the last entry.
            let last = lut[lut_len - 1];
            self.prev_angle = last.0;
            self.prev_y = last.1;
            self.prev_index = (lut_len - 1 ) as u16;
            return last.0;
        } else {
            // Backward traversal.
            // Ensure we don't underflow.
            if self.prev_index == 0 {
                let first = lut[0];
                self.prev_angle = first.0;
                self.prev_y = first.1;
                self.prev_index = 0;
                return first.0;
            }
            let mut index = self.prev_index;
            let mut prev = lut[index as usize];
            while index > 0 {
                index -= 1;
                let current = lut[index as usize];
                if current.1 == y_value {
                    self.prev_angle = current.0;
                    self.prev_y = current.1;
                    self.prev_index = index;
                    return current.0;
                } else if current.1 < y_value {
                    let angle = self.mean(current.0, prev.0);
                    return angle;
                }
                prev = current;
            }
            // If y_value is smaller than all LUT entries, return the first entry.
            let first = lut[0];
            self.prev_angle = first.0;
            self.prev_y = first.1;
            self.prev_index = 0;
            return first.0;
        }
    }
    
    fn mean(&self, angle_1: u8, angle_2: u8) -> u8 {
        (((angle_1 as u16) + (angle_2 as u16)) >> 1) as u8
    }

    pub fn lut_atan(&mut self, integer : Fix32_14) -> Fix16_7{
        let sign : bool = if integer > 0 {true} else {false};
        let y : u16 = (Fix16_7::from_num(integer.abs())).to_bits() as u16;
        match sign {
            true => return Fix16_7::from_bits(self.search_angle(y) as i16),
            false => return -Fix16_7::from_bits(self.search_angle(y) as i16)
        }
    }

}



