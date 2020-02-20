#include "animated_character.h"
#include "amc_util.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>


AnimatedCharacter::AnimatedCharacter(const std::string &asf_filename) :
    fps_(120.0), elapsed_since_last_frame_(0.0), current_frame_(0)
{
    LoadSkeleton(asf_filename);
}

AnimatedCharacter::AnimatedCharacter() :
    fps_(120.0), elapsed_since_last_frame_(0.0), current_frame_(0)
{
}

AnimatedCharacter::~AnimatedCharacter() {
}


void AnimatedCharacter::LoadSkeleton(const std::string &asf_filename) {
    skeleton_.LoadFromASF(asf_filename);
}


void AnimatedCharacter::Play(const MotionClip &motion_clip) {
    motion_queue_.clear();
    motion_queue_.push_back(motion_clip);
    current_frame_ = 0;
}


void AnimatedCharacter::Queue(const MotionClip &motion_clip) {
    if (motion_queue_.size() == 0) {
        Play(motion_clip);
    }
    else {
        motion_queue_.push_back(motion_clip);
    }
}


void AnimatedCharacter::ClearQueue() {
    motion_queue_.clear();
}


void AnimatedCharacter::OverlayClip(const MotionClip &clip, int num_transition_frames) {
    overlay_clip_ = clip;
    overlay_transition_frames_ = num_transition_frames;
    overlay_frame_ = 0;
}


void AnimatedCharacter::AdvanceAnimation(double dt) {
    if (motion_queue_.size() == 0) {
        pose_ = Pose();
    }
    else {
        elapsed_since_last_frame_ += dt;
        
        double frames_to_advance = fps_ * elapsed_since_last_frame_;
        double whole_frames;
        double frac = modf(frames_to_advance, &whole_frames);
        int nframes = (int)whole_frames;
        elapsed_since_last_frame_ = frac / fps_;
        
        for (int i=0; i<nframes; i++) {
            // advance the main motion track
            current_frame_++;
            // handle end case
            if (current_frame_ >= motion_queue_[0].size()) {
                // loop back to the first frame
                current_frame_ = 0;
                // if there are more motions in the queue then pop this one and goto the next
                if (motion_queue_.size() > 1) {
                    motion_queue_.erase(motion_queue_.begin());
                }
            }
            
            // advance the overlay clip if there is one
            if (overlay_clip_.size()) {
                overlay_frame_++;
                // handle end case
                if (overlay_frame_ >= overlay_clip_.size()) {
                    // done playing overlay, reset frame counter and clear the overlay clip
                    overlay_frame_ = 0;
                    overlay_clip_ = MotionClip();
                }
            }
            
            // update the pose based on new frames
            CalcCurrentPose();

            // add to the translation matrix for the case when relative root motion is used
            accum_translation_matrix_ = accum_translation_matrix_ * pose_.root_relative_translation();
        }
    }
}


void AnimatedCharacter::CalcCurrentPose() {
    if (!overlay_clip_.size()) {
        // no overaly track, motion is entirely from the base track (i.e., the motion queue)
        pose_ = motion_queue_[0][current_frame_];
    }
    else {
        // there is an active overlay track
        if (overlay_frame_ < overlay_transition_frames_) {
            // fade in the overlay
            float alpha = (float)overlay_frame_/(float)overlay_transition_frames_;
            pose_ = motion_queue_[0][current_frame_].Lerp(overlay_clip_[overlay_frame_], alpha);
        }
        else if (overlay_frame_ > overlay_clip_.size() - overlay_transition_frames_) {
            // fade out the overlay
            float alpha = (float)(overlay_clip_.size() - overlay_frame_)/(float)overlay_transition_frames_;
            pose_ = motion_queue_[0][current_frame_].Lerp(overlay_clip_[overlay_frame_], alpha);
        }
        else {
            // overlay is completely faded in, we don't see the base track at all
            pose_ = overlay_clip_[overlay_frame_];
        }
    }
}


Skeleton* AnimatedCharacter::skeleton_ptr() {
    return &skeleton_;
}


void AnimatedCharacter::set_fps(int fps) {
    fps_ = fps;
}


int AnimatedCharacter::fps() {
    return fps_;
}



void AnimatedCharacter::Draw(const Matrix4 &model_matrix, const Matrix4 &view_matrix, const Matrix4 &proj_matrix,
                             bool use_absolute_position)
{
    Matrix4 character_root_transform;
    if (use_absolute_position) {
        // set root position based on the absolute position in the mocap data
        character_root_transform = model_matrix * pose_.RootTransform();
    }
    else {
        // set root position based on the relative updates accumulated each frame
        character_root_transform = model_matrix * accum_translation_matrix_ * pose_.root_rotation();
    }
    
    for (int i=0; i<skeleton_.num_root_bones(); i++) {
        DrawBoneRecursive(skeleton_.root_bone(i), character_root_transform, view_matrix, proj_matrix);
    }
}


void AnimatedCharacter::DrawBoneRecursive(const std::string &bone_name, const Matrix4 &parent_transform,
                                          const Matrix4 &view_matrix, const Matrix4 &proj_matrix)
{
    // Step 1:  Draw this bone
    
    /** TODO: You will need to define a current transformation matrix for this bone that takes into account not just the parent_transform but also the local rotation of the bone due to the current pose.
     
        Think of the vertices that make up the geometry of each bone as being defined in "bone space", where the joint that the bone rotates around is located at the origin and the bone extends in the direction and length specified by the skeleton. (See Skeleton::BoneDirectionAndLength()).
     
        To determine which matrices need to be composed to create the current transformation matrix and the order to multiply them together, think about what needs to happen to each vertex of a cylinder defined in "bone space" in order to get the vertex to the correct position in 3D space.
     
        First, the vertex must be transformed into the bone's "rotation axis space" because the rotation axes are not guaranteed to line up perfectly with the bone's X,Y,Z axes.  The bone's rotation axes are a property of the skeleton -- they are set for each skeleton and do not change for each pose.  You can access a matrix that transforms from "bone space" to "rotation axis space" from the skeleton_ member variable.
     
        Second, now that the vertices are in the bone's "rotation axis space", the rotation from the character's current pose can be applied.  The current pose is stored in the pose_ member variable.

        Third, with the rotations applied relative to the appropriate rotation axes, the vertices must now be transformed back into regular "bone space".  At this point, the bone should be properly rotated based upon the current pose, but the vertices are still defined in "bone space" so they are close to the origin.
     
        Finally, the vertices need to be tranformed to the bone's parent space.
     
        To start, we give you a current transformation matrix (ctm) that only takes this last step into account.
    */
    
    // current transformation matrix for drawing each bone
    Matrix4 ctm = parent_transform;

    
    // Here is a good way to check your work -- draw the coordinate axes for each
    // bone.  To start, this will just draw the axes for the root node of the
    // character, but once you add the recursive call to draw the children, this
    // will draw the axes for each bone.
    //Matrix4 S = Matrix4::Scale(Vector3(0.15,0.15,0.15));
    //quick_shapes_.DrawAxes(ctm * S, view_matrix, proj_matrix);
    
    // draw dancing stick figures
    Vector3 bone = skeleton_.BoneDirectionAndLength(bone_name);
    Matrix4 body_transform = ctm * skeleton_.RotAxesSpaceToBoneSpace(bone_name) * pose_.JointRotation(bone_name) * skeleton_.BoneSpaceToRotAxesSpace(bone_name);
    
    Color red = Color(1, 0, 0);
    Color black = Color(0, 0, 0);
    
    //Point3 end = Point3::Zero() + bone;
    //quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);

    // TODO: Eventually, you'll want to draw something different depending on which part
    // of the body is being drawn.  An if statement like this is an easy way to do that.
    if (bone_name == "lhipjoint" || bone_name == "rhipjoint") {
        Point3 end = Point3::Zero() + bone;
        quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);
    }
    
    if (bone_name == "lfemur" || bone_name == "rfemur") {
        Point3 end = Point3::Zero() + bone;
        quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);
    }
    
    if (bone_name == "ltibia" || bone_name == "rtibia") {
        Point3 end = Point3::Zero() + bone;
        quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);
    }
    
    if (bone_name == "lfoot" || bone_name == "rfoot") {
        Matrix4 foot_rotation = Matrix4::RotationZ(-M_PI / 2);
        Matrix4 foot_scaling = Matrix4::Scale(Vector3(0.05, 0.05, 0.09));
        Matrix4 foot_translation = Matrix4::Translation(bone / 2 + Vector3(0, -0.01, 0.07));
        Matrix4 foot_transformation = foot_translation * foot_scaling * foot_rotation;
        Point3 end = Point3::Zero() + bone;
        
        quick_shapes_.DrawSphere(body_transform * foot_transformation, view_matrix, proj_matrix, red);
        quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);
    }
    
    if (bone_name == "ltoes" || bone_name == "rtoes") {
        Point3 end = Point3::Zero() + bone;
        quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);
    }
    
    if (bone_name == "lowerback") {
        //Matrix4 skirt_scaling = Matrix4::Scale(Vector3(0.4, 0.17, 0.4));
        //Matrix4 skirt_translation = Matrix4::Translation(Vector3(0, -0.55, 0));
        //quick_shapes_.DrawCone(ctm * skirt_scaling * skirt_translation, view_matrix, proj_matrix, Color(1, 1, 1));
        
        quick_shapes_.DrawSphere(body_transform * Matrix4::Translation(-bone / 2.0) * Matrix4::Scale(Vector3(0.08, 0.24, 0.08)), view_matrix, proj_matrix, red);
    }
    
    if (bone_name == "upperback") {
        Point3 end = Point3::Zero() + bone;
        quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);
        
        // draw the second set of hands
        DrawBoneRecursive("lclavicle", body_transform * skeleton_.BoneSpaceToChildrenSpace(bone_name) * Matrix4::Scale(Vector3(0.6, 0.6, 0.6)), view_matrix, proj_matrix);
        DrawBoneRecursive("rclavicle", body_transform * skeleton_.BoneSpaceToChildrenSpace(bone_name) * Matrix4::Scale(Vector3(0.6, 0.6, 0.6)), view_matrix, proj_matrix);
    }
    
    if (bone_name == "thorax") {
        Matrix4 thorax_scaling = Matrix4::Scale(Vector3(0.06, 0.12, 0.06));
        Matrix4 thorax_translation = Matrix4::Translation(bone / 2);
        quick_shapes_.DrawSphere(body_transform * thorax_translation * thorax_scaling, view_matrix, proj_matrix, red);
    }
    
    if (bone_name == "lowerneck" || bone_name == "upperneck") {
        Matrix4 neck_scaling = Matrix4::Scale(Vector3(0.15, 0.15, 0.15));
        Point3 neck_start = Point3(0, 0.7, 0);
        Point3 neck_end = Point3(0, -0.7, 0);
        
        quick_shapes_.DrawLineSegment(ctm * neck_scaling, view_matrix, proj_matrix, black, neck_start, neck_end, 0.1);
    }
    
    if (bone_name == "head") {
        Matrix4 head_scaling = Matrix4::Scale(Vector3(0.07, 0.13, 0.07));
        Matrix4 head_translation = Matrix4::Translation(bone / 2);
        Matrix4 head_rotation = Matrix4::RotationX(-M_PI / 4);
        
        Matrix4 translation = Matrix4::Translation(Vector3(0.0, 0.11, 0.0));
        Matrix4 right_rotation = Matrix4::RotationZ(0.3);
        Matrix4 left_rotation = Matrix4::RotationZ(-0.3);
        
        // head
        quick_shapes_.DrawSphere(body_transform * head_translation * head_rotation * head_scaling, view_matrix, proj_matrix, red);
        
        // right
        quick_shapes_.DrawLineSegment(body_transform * right_rotation * translation, view_matrix, proj_matrix, black, Point3::Zero(), Point3(0, 0.25, 0.1), 0.007);
        quick_shapes_.DrawLineSegment(body_transform * right_rotation * translation, view_matrix, proj_matrix, black, Point3(0, 0.25, 0.1), Point3(0, 0.25, 0.17), 0.007);
        
        // left
        quick_shapes_.DrawLineSegment(body_transform * left_rotation * translation, view_matrix, proj_matrix, black, Point3::Zero(), Point3(0, 0.25, 0.1), 0.007);
        quick_shapes_.DrawLineSegment(body_transform * left_rotation * translation, view_matrix, proj_matrix, black, Point3(0, 0.25, 0.1), Point3(0, 0.25, 0.17), 0.007);
        }
    
    if (bone_name == "lclavicle" || bone_name == "rclavicle") {
        Matrix4 clavicle_scaling = Matrix4::Scale(Vector3(0.07, 0.1, 0.07));
        Point3 end = Point3::Zero() + bone;
        
        quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);
        quick_shapes_.DrawSphere(ctm * clavicle_scaling, view_matrix, proj_matrix, red);
    }
    
    if (bone_name == "lhumerus" || bone_name == "rhumerus" || bone_name == "lradius" || bone_name == "rradius") {
        Point3 end = Point3::Zero() + bone;
        quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);
        
        if (bone_name == "lhumerus" || bone_name == "rhumerus") {
            Matrix4 humerus_scaling = Matrix4::Scale(Vector3(0.15, 0.06, 0.06));
            quick_shapes_.DrawSphere(ctm * humerus_scaling, view_matrix, proj_matrix, red);
        }
    }
    
    if (bone_name == "lwrist" || bone_name == "rwrist") {
        Point3 end = Point3::Zero() + bone;
        quick_shapes_.DrawLineSegment(body_transform, view_matrix, proj_matrix, black, Point3::Zero(), end, 0.01);
    }
    
    if (bone_name == "lhand" || bone_name == "rhand" || bone_name == "lthumb" || bone_name == "rthumb" || bone_name == "rfingers" || bone_name == "lfingers") {
        
        Matrix4 right_part_translation = Matrix4::Translation(Vector3(-0.02, 0, 0));
        
        if (bone_name == "lhand") {
            Matrix4 hand_translation = Matrix4::Scale(Vector3(0.05, 0.05, 0.05));
            quick_shapes_.DrawSphere(ctm * hand_translation, view_matrix, proj_matrix, red);
        }
        
        else if (bone_name == "rhand") {
            Matrix4 hand_translation = Matrix4::Scale(Vector3(0.05, 0.05, 0.05));
            quick_shapes_.DrawSphere(ctm * right_part_translation * hand_translation, view_matrix, proj_matrix, red);
        }
        
        else if (bone_name == "lthumb") {
            Matrix4 thumb_transform = Matrix4::Scale(Vector3(0.02, 0.02, 0.02)) * Matrix4::Translation(Vector3(1, 0, 0));
            quick_shapes_.DrawSphere(ctm * thumb_transform, view_matrix, proj_matrix, red);
        }
        
        else if (bone_name == "rthumb") {
            Matrix4 thumb_transform = Matrix4::Scale(Vector3(0.02, 0.02, 0.02)) * Matrix4::Translation(Vector3(1, 0, 0));
            quick_shapes_.DrawSphere(ctm * right_part_translation * thumb_transform, view_matrix, proj_matrix, red);
        }
        
        else if (bone_name == "lfingers") {
            Matrix4 finger_scaling = Matrix4::Scale(Vector3(0.01, 0.01, 0.01));
            quick_shapes_.DrawSphere(ctm * finger_scaling, view_matrix, proj_matrix, red);
        }
        
        else {
            Matrix4 finger_scaling = Matrix4::Scale(Vector3(0.01, 0.01, 0.01));
            quick_shapes_.DrawSphere(ctm * right_part_translation * finger_scaling, view_matrix, proj_matrix, red);
        }
    }
    
    
    // Step 2: Draw the bone's children
    /**
     
    // TODO: Determining the proper child_root_transform is the key here.  It depends on the
    // current transformation matrix, but you also need to take into account the
    // direction and length of the bone in order to reach the root of the children.
    Matrix4 child_root_transform = ????;
     
    for (int i=0; i<skeleton_.num_children(bone_name); i++) {
        DrawBoneRecursive(skeleton_.child_bone(bone_name, i), child_root_transform, view_matrix, proj_matrix);
    }
    **/
    
    // set the child_root_transform matrix for drawing the bones
    Matrix4 child_root_transform = body_transform * skeleton_.BoneSpaceToChildrenSpace(bone_name);
    
    // draw all of the bones in the skeleton
    for (int i = 0; i < skeleton_.num_children(bone_name); i++)
        DrawBoneRecursive(skeleton_.child_bone(bone_name, i), child_root_transform, view_matrix, proj_matrix);
}



